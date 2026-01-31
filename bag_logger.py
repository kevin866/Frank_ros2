#!/usr/bin/env python3
import os, csv, yaml
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import numpy as np

# --------- CONFIG ----------
BAG_DIR = "rr_run1"        # folder containing metadata.yaml
OUT_DIR = "plots_csv"

topics_to_record = [
   '/resolved_rate_controller/ee_twist',  # command going into RR
    '/ee_pose',                            # feedback
    '/joint_states',                       # joints
]
# --------------------------

WS_ROOT = os.path.dirname(os.path.abspath(__file__))
BAG_DIR = os.path.join(WS_ROOT, BAG_DIR)
OUT_DIR = os.path.join(WS_ROOT, OUT_DIR)
os.makedirs(OUT_DIR, exist_ok=True)

# ---- load metadata for storage id ----
meta_path = os.path.join(BAG_DIR, "metadata.yaml")
with open(meta_path, "r") as f:
    meta = yaml.safe_load(f)
storage_id = meta["rosbag2_bagfile_information"]["storage_identifier"]
print(f"[INFO] Bag: {BAG_DIR} storage_id={storage_id}")

# ---- open reader ----
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG_DIR, storage_id=storage_id),
    rosbag2_py.ConverterOptions("", "")
)

topics_and_types = reader.get_all_topics_and_types()
bag_type = {t.name: t.type for t in topics_and_types}

# verify topics exist
for topic in topics_to_record:
    if topic not in bag_type:
        print(f"[WARN] Topic missing in bag: {topic}")
print("[INFO] Bag topic scan complete.")

# message classes for requested topics that exist
msg_cls = {}
for topic in topics_to_record:
    if topic in bag_type:
        msg_cls[topic] = get_message(bag_type[topic])

# ---------- CSV helpers ----------
writers = {}
files = {}

def sanitize_topic_to_filename(topic: str) -> str:
    # "/a/b/c" -> "a__b__c.csv"
    name = topic.strip("/").replace("/", "__")
    if not name:
        name = "root"
    return f"{name}.csv"

def open_writer_for_topic(topic: str, header):
    filename = sanitize_topic_to_filename(topic)
    path = os.path.join(OUT_DIR, filename)
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(header)
    files[topic] = f
    writers[topic] = w

# ---------- Handlers ----------
def setup_writer_and_row_fn(topic: str, msg_type: str):
    """
    Returns: (row_fn, header)
    row_fn(msg, t_sec) -> list matching header
    """
    # PoseStamped
    if msg_type == "geometry_msgs/msg/PoseStamped":
        header = ["t","x","y","z","qx","qy","qz","qw"]
        def row_fn(msg, t):
            p = msg.pose.position
            q = msg.pose.orientation
            return [t, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        return row_fn, header

    # TwistStamped
    if msg_type == "geometry_msgs/msg/TwistStamped":
        header = ["t","vx","vy","vz","wx","wy","wz"]
        def row_fn(msg, t):
            lin = msg.twist.linear
            ang = msg.twist.angular
            return [t, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
        return row_fn, header

    # Twist (non-stamped)
    if msg_type == "geometry_msgs/msg/Twist":
        header = ["t","vx","vy","vz","wx","wy","wz"]
        def row_fn(msg, t):
            lin = msg.linear
            ang = msg.angular
            return [t, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
        return row_fn, header

    # JointState
    if msg_type == "sensor_msgs/msg/JointState":
        # variable-length arrays; simplest is store as semicolon-joined strings
        header = ["t", "name", "position", "velocity", "effort"]
        def row_fn(msg, t):
            name = ";".join(msg.name)
            pos = ";".join([str(x) for x in msg.position])
            vel = ";".join([str(x) for x in msg.velocity])
            eff = ";".join([str(x) for x in msg.effort])
            return [t, name, pos, vel, eff]
        return row_fn, header

    # Image (depth). Writing every pixel to CSV is usually unusable.
    # This writes a compact summary per frame: size + encoding + step.
    if msg_type == "sensor_msgs/msg/Image":
        header = ["t", "depth_center", "depth_r", "depth_l", "depth_d", "depth_u"]

        def row_fn(msg, t):
            depth = depth_msg_to_numpy(msg)
            h, w = depth.shape
            cy, cx = h // 2, w // 2

            return [
                t,
                float(depth[cy, cx]),          # center
                float(depth[cy, cx + 20]),     # right
                float(depth[cy, cx - 20]),     # left
                float(depth[cy + 20, cx]),     # down
                float(depth[cy - 20, cx]),     # up
            ]

        return row_fn, header


    # Fallback: warn and skip
    return None, None

def depth_summary_row(msg, t):
    depth = depth_msg_to_numpy(msg)
    h, w = depth.shape

    cx, cy = w // 2, h // 2

    return [
        t,
        depth[cy, cx],           # center
        depth[cy, cx + 20],      # right
        depth[cy, cx - 20],      # left
        depth[cy + 20, cx],      # down
        depth[cy - 20, cx],      # up
    ]


def depth_msg_to_numpy(msg):
    if msg.encoding == "16UC1":
        dtype = np.uint16
        depth = np.frombuffer(msg.data, dtype=dtype)
        depth = depth.reshape((msg.height, msg.width))
        depth = depth.astype(np.float32) / 1000.0  # mm → meters
        return depth

    elif msg.encoding == "32FC1":
        dtype = np.float32
        depth = np.frombuffer(msg.data, dtype=dtype)
        depth = depth.reshape((msg.height, msg.width))
        return depth

    else:
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")



# Build per-topic row functions + open CSVs
row_fn = {}
for topic in topics_to_record:
    if topic not in bag_type:
        continue
    ttype = bag_type[topic]
    fn, header = setup_writer_and_row_fn(topic, ttype)
    if fn is None:
        print(f"[WARN] No handler for {topic} type={ttype}. Skipping.")
        continue
    row_fn[topic] = fn
    open_writer_for_topic(topic, header)
    print(f"[INFO] Recording {topic} ({ttype}) -> {sanitize_topic_to_filename(topic)}")

# ---- read bag ----
n = {topic: 0 for topic in writers.keys()}

while reader.has_next():
    topic, data, t_ns = reader.read_next()
    if topic not in writers:
        continue
    if topic not in msg_cls:
        continue
    msg = deserialize_message(data, msg_cls[topic])
    t = t_ns * 1e-9
    writers[topic].writerow(row_fn[topic](msg, t))
    n[topic] += 1

# close files
for topic, f in files.items():
    f.close()

for topic, cnt in n.items():
    print(f"[INFO] Wrote {cnt:6d} rows for {topic}")
print(f"[INFO] CSVs in: {OUT_DIR}")
