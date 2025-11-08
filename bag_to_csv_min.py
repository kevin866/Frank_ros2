#!/usr/bin/env python3
import os, csv, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Folder that CONTAINS my_test_run_0.db3 (not the .db3 file itself)
BAG_DIR = '/home/frank/frank_ws/ombot_run'

topics = [
    '/mecanum_controller/reference',
    '/vrpn_mocap/RigidBody_1/pose',
    '/resolved_rate_controller/ee_twist',
    '/goal_pose',
]

# Open bag (sqlite3)
storage = rosbag2_py.StorageOptions(uri=BAG_DIR, storage_id='sqlite3')
converter = rosbag2_py.ConverterOptions('', '')
reader = rosbag2_py.SequentialReader()
reader.open(storage, converter)

# Map topic -> type
info = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in info}

# Build msg class map just for our topics that exist
msg_types = {}
missing = []
for name in topics:
    typ = type_map.get(name)
    if not typ:
        missing.append(name)
        continue
    msg_types[name] = get_message(typ)

if missing:
    print("Warning: these topics are not in the bag:", *missing, sep="\n- ")

# CSV writers
def open_writer(path, header):
    f = open(path, 'w', newline='')
    w = csv.writer(f); w.writerow(header)
    return f, w

writers, files = {}, {}
for topic, typ in msg_types.items():
    base = topic.strip('/').replace('/', '_')
    is_twist = 'Twist' in type_map[topic]
    hdr = ['t','vx','vy','vz','wx','wy','wz'] if is_twist else ['t','x','y','z','qx','qy','qz','qw']
    f, w = open_writer(f'{base}.csv', hdr)
    files[topic] = f
    writers[topic] = w

count = 0
while reader.has_next():
    topic, data, t_nsec = reader.read_next()
    if topic not in msg_types:
        continue
    msg = deserialize_message(data, msg_types[topic])
    t = t_nsec / 1e9

    # PoseStamped
    if hasattr(msg, 'pose'):
        p, q = msg.pose.position, msg.pose.orientation
        writers[topic].writerow([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    # TwistStamped
    elif hasattr(msg, 'twist'):
        lin, ang = msg.twist.linear, msg.twist.angular
        writers[topic].writerow([t, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])

    count += 1

for f in files.values():
    f.close()

print(f"Done. Wrote CSV files for {len(writers)} topics to: {os.getcwd()}")
