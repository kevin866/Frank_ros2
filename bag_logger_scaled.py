#!/usr/bin/env python3
import os, csv, yaml
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# -------- config --------
BAG_DIR = "ombot_run1"        # folder containing metadata.yaml
OUT_DIR = "plots_csv"

# Which message types to convert (add more later if you want)
SUPPORTED = {
    "geometry_msgs/msg/PoseStamped": "pose",
    "geometry_msgs/msg/TwistStamped": "twist",
    "sensor_msgs/msg/JointState": "joint_state",
    "std_msgs/msg/Float64": "float64",
    "ombot_msgs/msg/WholeBodyCmd": "wb_cmd",

}


WS_ROOT = os.path.dirname(os.path.abspath(__file__))
BAG_DIR = os.path.join(WS_ROOT, BAG_DIR)
OUT_DIR = os.path.join(WS_ROOT, OUT_DIR)
os.makedirs(OUT_DIR, exist_ok=True)

def safe_name(topic: str) -> str:
    # "/a/b" -> "a__b"
    return topic.strip("/").replace("/", "__")

def open_writer(topic: str, kind: str, joint_names=None):
    fname = f"{safe_name(topic)}.csv"
    path = os.path.join(OUT_DIR, fname)
    f = open(path, "w", newline="")
    w = csv.writer(f)

    if kind == "pose":
        w.writerow(["t","x","y","z","qx","qy","qz","qw"])
    elif kind == "twist":
        w.writerow(["t","vx","vy","vz","wx","wy","wz"])
    elif kind == "joint_state":
        # Wide format: t, then pos/vel/eff per joint name
        if not joint_names:
            # header will be written later once we see the first message
            pass
        else:
            header = ["t"]
            for j in joint_names:
                header += [f"pos_{j}", f"vel_{j}", f"eff_{j}"]
            w.writerow(header)
    elif kind in ("float32", "float64"):
        w.writerow(["t", "data"])
    elif kind == "wb_cmd":
        w.writerow([
            "t",
            "bvx","bvy","bwz",
            "ee_vx","ee_vy","ee_vz",
            "ee_wx","ee_wy","ee_wz",
            "valid"
        ])
    else:
        raise ValueError(f"Unknown kind: {kind}")

    return w, f


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

# ---- build per-topic converters dynamically ----
msg_cls = {}
writers = {}
files = {}
kind_of = {}
n = {}

converted_topics = []
skipped_topics = []

for topic, tstr in bag_type.items():
    if tstr in SUPPORTED:
        kind = SUPPORTED[tstr]
        kind_of[topic] = kind
        msg_cls[topic] = get_message(tstr)
        w, f = open_writer(topic, kind)
        writers[topic] = w
        files[topic] = f
        n[topic] = 0
        converted_topics.append((topic, tstr))
    else:
        skipped_topics.append((topic, tstr))

print("[INFO] Will convert topics:")
for topic, tstr in converted_topics:
    print(f"  - {topic}  ({tstr})")

if skipped_topics:
    print("[INFO] Skipping unsupported topics:")
    for topic, tstr in skipped_topics:
        print(f"  - {topic}  ({tstr})")

# ---- read bag ----
while reader.has_next():
    topic, data, t_ns = reader.read_next()
    if topic not in writers:
        continue

    msg = deserialize_message(data, msg_cls[topic])
    t = t_ns * 1e-9

    if kind_of[topic] == "pose":
        p = msg.pose.position
        q = msg.pose.orientation
        writers[topic].writerow([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    elif kind_of[topic] == "twist":
        lin = msg.twist.linear
        ang = msg.twist.angular
        writers[topic].writerow([t, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])
    elif kind_of[topic] in ("float32", "float64"):
        writers[topic].writerow([t, msg.data])
    elif kind_of[topic] == "wb_cmd":
        ee = msg.ee
        writers[topic].writerow([
            t,
            float(msg.bvx), float(msg.bvy), float(msg.bwz),
            float(ee.linear.x), float(ee.linear.y), float(ee.linear.z),
            float(ee.angular.x), float(ee.angular.y), float(ee.angular.z),
            int(msg.valid)
        ])



    n[topic] += 1

# ---- close files ----
for f in files.values():
    f.close()

for topic, cnt in n.items():
    print(f"[INFO] Wrote {cnt:6d} rows for {topic}")
print(f"[INFO] CSVs in: {OUT_DIR}")
