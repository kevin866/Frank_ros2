#!/usr/bin/env python3
import os
import csv
import yaml

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# -------- config --------
BAG_DIR = "ombot_run1"        # folder containing metadata.yaml
OUT_DIR = "plots_csv"

SUPPORTED = {
    "geometry_msgs/msg/PoseStamped": "pose",
    "geometry_msgs/msg/TwistStamped": "twist",
    "sensor_msgs/msg/JointState": "joint_state",
    "std_msgs/msg/Float64": "float",
    "std_msgs/msg/Float32": "float",
    "ombot_msgs/msg/WholeBodyCmd": "wb_cmd",
    "geometry_msgs/msg/Vector3Stamped": "vector3",
}

WS_ROOT = os.path.dirname(os.path.abspath(__file__))
BAG_DIR = os.path.join(WS_ROOT, BAG_DIR)
OUT_DIR = os.path.join(WS_ROOT, OUT_DIR)
os.makedirs(OUT_DIR, exist_ok=True)


def safe_name(topic: str) -> str:
    # "/a/b" -> "a__b"
    return topic.strip("/").replace("/", "__")


def stamp_to_sec(stamp) -> float:
    # builtin_interfaces/msg/Time
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def open_writer(topic: str, kind: str):
    fname = f"{safe_name(topic)}.csv"
    path = os.path.join(OUT_DIR, fname)
    f = open(path, "w", newline="")
    w = csv.writer(f)

    if kind == "pose":
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
    elif kind == "twist":
        w.writerow(["t", "vx", "vy", "vz", "wx", "wy", "wz"])
    elif kind == "joint_state":
        # header written when we see first message (need msg.name)
        pass
    elif kind == "float":
        w.writerow(["t", "data"])
    elif kind == "wb_cmd":
        w.writerow([
            "t",
            "bvx", "bvy", "bwz",
            "ee_vx", "ee_vy", "ee_vz",
            "ee_wx", "ee_wy", "ee_wz",
            "valid",
        ])
    elif kind == "vector3":
        w.writerow(["t", "frame_id", "x", "y", "z"])
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
row_counts = {}

JS_HEADER_WRITTEN = set()   # topics whose JointState header has been written
JS_ORDER = {}               # topic -> list of joint names (column order)

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
        row_counts[topic] = 0
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
    kind = kind_of[topic]
    w = writers[topic]

    # default time = bag time
    t_bag = float(t_ns) * 1e-9

    if kind == "pose":
        # PoseStamped has header stamp
        t = stamp_to_sec(msg.header.stamp) if hasattr(msg, "header") else t_bag
        p = msg.pose.position
        q = msg.pose.orientation
        w.writerow([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    elif kind == "twist":
        t = stamp_to_sec(msg.header.stamp) if hasattr(msg, "header") else t_bag
        lin = msg.twist.linear
        ang = msg.twist.angular
        w.writerow([t, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])

    elif kind == "float":
        w.writerow([t_bag, float(msg.data)])

    elif kind == "wb_cmd":
        # time source depends on your msg definition; use bag time unless it has header
        t = stamp_to_sec(msg.header.stamp) if hasattr(msg, "header") else t_bag
        ee = msg.ee
        w.writerow([
            t,
            float(msg.bvx), float(msg.bvy), float(msg.bwz),
            float(ee.linear.x), float(ee.linear.y), float(ee.linear.z),
            float(ee.angular.x), float(ee.angular.y), float(ee.angular.z),
            int(msg.valid),
        ])

    elif kind == "vector3":
        t = stamp_to_sec(msg.header.stamp) if hasattr(msg, "header") else t_bag
        w.writerow([t, msg.header.frame_id, msg.vector.x, msg.vector.y, msg.vector.z])

    elif kind == "joint_state":
        t = stamp_to_sec(msg.header.stamp) if hasattr(msg, "header") else t_bag

        # Write header once, using first seen msg.name order
        if topic not in JS_HEADER_WRITTEN:
            JS_ORDER[topic] = list(msg.name)
            header = ["t"]
            for j in JS_ORDER[topic]:
                header += [f"pos_{j}", f"vel_{j}", f"eff_{j}"]
            w.writerow(header)
            JS_HEADER_WRITTEN.add(topic)

        name_to_i = {nm: i for i, nm in enumerate(msg.name)}
        order = JS_ORDER[topic]

        def get_arr(arr, i):
            return float(arr[i]) if arr is not None and i < len(arr) else 0.0

        row = [t]
        for j in order:
            i = name_to_i.get(j, None)
            if i is None:
                row += [0.0, 0.0, 0.0]
            else:
                row += [
                    get_arr(msg.position, i),
                    get_arr(msg.velocity, i),
                    get_arr(msg.effort, i),
                ]
        w.writerow(row)

    row_counts[topic] += 1

# ---- close files ----
for f in files.values():
    f.close()

for topic, cnt in row_counts.items():
    print(f"[INFO] Wrote {cnt:6d} rows for {topic}")
print(f"[INFO] CSVs in: {OUT_DIR}")