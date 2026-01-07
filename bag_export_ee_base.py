#!/usr/bin/env python3
import os, csv, yaml
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

BAG_DIR = "ombot_run1"        # folder containing metadata.yaml
OUT_DIR = "plots_csv"

TOPICS = {
    "/ee_pose": "geometry_msgs/msg/PoseStamped",
    "/ee_desired_pose": "geometry_msgs/msg/PoseStamped",
    "/base_desired_twist": "geometry_msgs/msg/TwistStamped",
    "/wb_resolved_rate_controller/desired_twist": "geometry_msgs/msg/TwistStamped",
    "/ee_desired_twist": "geometry_msgs/msg/TwistStamped",
}

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
for topic in TOPICS:
    if topic not in bag_type:
        print(f"[WARN] Topic missing in bag: {topic}")
print("[INFO] Topics found OK.")

# message classes
msg_cls = {}
for topic, _ in TOPICS.items():
    if topic in bag_type:
        msg_cls[topic] = get_message(bag_type[topic])

# open CSV writers
writers = {}
files = {}

def open_writer(name, header):
    path = os.path.join(OUT_DIR, name)
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(header)
    files[name] = f
    return w

writers["/ee_pose"] = open_writer("ee_pose.csv",
    ["t","x","y","z","qx","qy","qz","qw"])
writers["/ee_desired_pose"] = open_writer("ee_desired_pose.csv",
    ["t","x","y","z","qx","qy","qz","qw"])
writers["/base_desired_twist"] = open_writer("base_desired_twist.csv",
    ["t","vx","vy","vz","wx","wy","wz"])
writers["/wb_resolved_rate_controller/desired_twist"] = open_writer("desired_twist_cmd.csv",
    ["t","vx","vy","vz","wx","wy","wz"])
writers["/ee_desired_twist"] = open_writer("ee_twist_ff.csv",
    ["t","vx","vy","vz","wx","wy","wz"])

# ---- read bag ----
n = {k: 0 for k in writers.keys()}

while reader.has_next():
    topic, data, t_ns = reader.read_next()
    if topic not in writers:
        continue
    if topic not in msg_cls:
        continue
    msg = deserialize_message(data, msg_cls[topic])
    t = t_ns * 1e-9

    if "PoseStamped" in bag_type[topic]:
        p = msg.pose.position
        q = msg.pose.orientation
        writers[topic].writerow([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    else:
        lin = msg.twist.linear
        ang = msg.twist.angular
        writers[topic].writerow([t, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])

    n[topic] += 1

for name, f in files.items():
    f.close()

for topic, cnt in n.items():
    print(f"[INFO] Wrote {cnt:6d} rows for {topic}")
print(f"[INFO] CSVs in: {OUT_DIR}")
