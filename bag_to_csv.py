#!/usr/bin/env python3

import os
import csv
import yaml

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# =======================
# CONFIG
# =======================

BAG_DIR = "joint_excitation_run"   # Folder containing metadata.yaml
OUT_CSV = "src/ombot_sysid/sysid_data.csv"

TOPIC_TAU = "/joint_excitation_controller/tau_cmd"
TOPIC_JS  = "/joint_states"


BAG_DIR = "ombot_run1"
OUT_DIR = "plots_csv"   # folder to write many CSVs

TOPICS = {
  # End-effector
  "/ee_pose": "pose",                                  # PoseStamped (usually)
  "/wb_resolved_rate_controller/ee_twist": "twist",     # TwistStamped

  # Base
  "/mecanum_controller/reference": "twist",             # TwistStamped (you wrote TwistStamped)

  # Tracking / mocap
  "/vrpn_mocap/RigidBody_1/pose": "pose",               # PoseStamped
  "/vrpn_mocap/RigidBody_2/pose": "pose",               # PoseStamped

  # Goal
  "/goal_pose": "pose",                                 # PoseStamped

  # Robot state
  "/joint_states": "joint_state",                       # sensor_msgs/JointState
}


# Optional constant time shift for tau_cmd (in seconds)
# Positive = move tau later in time, Negative = earlier
SHIFT_TAU_SEC = 0.0

# Workspace root = directory where this script lives
WS_ROOT = os.path.dirname(os.path.abspath(__file__))

BAG_DIR = os.path.join(WS_ROOT, BAG_DIR)
OUT_CSV = os.path.join(WS_ROOT, OUT_CSV)

# =======================
# 1) Load metadata.yaml
# =======================

metadata_path = os.path.join(BAG_DIR, "metadata.yaml")
if not os.path.exists(metadata_path):
    raise FileNotFoundError(f"metadata.yaml not found at {metadata_path}")

with open(metadata_path, "r") as f:
    meta = yaml.safe_load(f)

info = meta["rosbag2_bagfile_information"]
storage_id = info["storage_identifier"]

print(f"[INFO] Bag dir:    {BAG_DIR}")
print(f"[INFO] Storage ID: {storage_id}")

# =======================
# 2) Open rosbag
# =======================

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_DIR, storage_id=storage_id)
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

topics_and_types = reader.get_all_topics_and_types()
print("[INFO] Topics in bag:")
for t in topics_and_types:
    print(f"  - {t.name} ({t.type})")

def find_topic_type(name):
    for t in topics_and_types:
        if t.name == name:
            return t.type
    return None

topic_type_tau = find_topic_type(TOPIC_TAU)
topic_type_js  = find_topic_type(TOPIC_JS)

if topic_type_tau is None:
    raise RuntimeError(f"Topic {TOPIC_TAU} not found in bag.")
if topic_type_js is None:
    raise RuntimeError(f"Topic {TOPIC_JS} not found in bag.")

msg_type_tau = get_message(topic_type_tau)
msg_type_js  = get_message(topic_type_js)

print(f"[INFO] Exporting {TOPIC_TAU} ({topic_type_tau})")
print(f"[INFO] Exporting {TOPIC_JS} ({topic_type_js})")

# =======================
# 3) Read messages (raw times)
# =======================

tau_samples = []   # list of (t_sec_raw, [tau...])
js_samples  = []   # list of (t_sec_raw, row_dict_with_joint_data)

joint_names = None

while reader.has_next():
    topic, data, t = reader.read_next()
    t_sec = t / 1e9  # raw time in seconds (bag time)

    # ---- tau_cmd ----
    if topic == TOPIC_TAU:
        msg = deserialize_message(data, msg_type_tau)

        if not hasattr(msg, "data"):
            raise RuntimeError(
                f"Tau message type {topic_type_tau} has no 'data' field; "
                f"adjust the script for its structure."
            )

        tau_list = list(msg.data)
        tau_samples.append((t_sec, tau_list))

    # ---- joint_states ----
    elif topic == TOPIC_JS:
        msg = deserialize_message(data, msg_type_js)

        if not hasattr(msg, "name") or not hasattr(msg, "position"):
            raise RuntimeError(
                f"JointState message type {topic_type_js} does not look like sensor_msgs/msg/JointState"
            )

        if joint_names is None:
            joint_names = list(msg.name)
            print(f"[INFO] JointState names/order: {joint_names}")

        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        row = {"time_sec": t_sec}

        for jname in joint_names:
            idx = name_to_idx.get(jname)

            # pos
            if idx is not None and idx < len(msg.position):
                row[f"{jname}_pos"] = msg.position[idx]
            else:
                row[f"{jname}_pos"] = float("nan")

            # vel
            if idx is not None and idx < len(msg.velocity):
                row[f"{jname}_vel"] = msg.velocity[idx]
            else:
                row[f"{jname}_vel"] = float("nan")

            # effort
            if idx is not None and idx < len(msg.effort):
                row[f"{jname}_eff"] = msg.effort[idx]
            else:
                row[f"{jname}_eff"] = float("nan")

        js_samples.append((t_sec, row))

print(f"[INFO] Read {len(tau_samples)} tau_cmd messages")
print(f"[INFO] Read {len(js_samples)} joint_states messages")

if not tau_samples or not js_samples:
    raise RuntimeError("Need at least some messages from both topics to build combined CSV.")

# =======================
# 4) Apply time shift to tau_cmd and normalize time
# =======================

# Shift tau times by constant offset (for delay compensation)
tau_samples_shifted = [(t + SHIFT_TAU_SEC, taus) for (t, taus) in tau_samples]

# Determine global t0 after shifting
min_tau_t = min(t for (t, _) in tau_samples_shifted)
min_js_t  = min(t for (t, _) in js_samples)
t0 = min(min_tau_t, min_js_t)

print(f"[INFO] Time shift for tau_cmd: {SHIFT_TAU_SEC} s")
print(f"[INFO] Global t0 (after shift) = {t0:.9f} s → will be mapped to 0.0")

# Sort by time just in case (rosbag should already be sorted)
tau_samples_shifted.sort(key=lambda x: x[0])
js_samples.sort(key=lambda x: x[0])

# =======================
# 5) Merge onto joint_states timeline (ZOH for tau_cmd)
# =======================

# Determine max tau dimension
max_tau_len = max(len(taus) for (_, taus) in tau_samples_shifted)

combined_rows = []

tau_idx = -1  # index of last tau sample whose time <= current js time

for (t_js_raw, js_row) in js_samples:
    t_js = t_js_raw  # raw time
    t_js_norm = t_js - t0  # normalize

    # Advance tau index while next tau sample is still <= current js time
    while (tau_idx + 1) < len(tau_samples_shifted) and tau_samples_shifted[tau_idx + 1][0] <= t_js:
        tau_idx += 1

    # Get current tau_cmd (ZOH)
    if tau_idx >= 0:
        t_tau_curr, tau_curr = tau_samples_shifted[tau_idx]
    else:
        tau_curr = None  # no tau yet

    combined = {}

    # Normalized time
    combined["time_sec"] = t_js_norm

    # Tau columns
    for i in range(max_tau_len):
        if tau_curr is not None and i < len(tau_curr):
            v = tau_curr[i]
        else:
            v = float("nan")
        combined[f"tau_{i}"] = v

    # Joint state columns
    for k, v in js_row.items():
        if k == "time_sec":
            continue  # we already added normalized time
        combined[k] = v

    combined_rows.append(combined)

print(f"[INFO] Combined rows: {len(combined_rows)}")

# =======================
# 6) Write single CSV (high precision)
# =======================

os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)

# Order columns: time_sec, tau_*, then the rest
fieldnames = sorted(
    combined_rows[0].keys(),
    key=lambda k: (
        k != "time_sec",                # time_sec first
        not k.startswith("tau_"),       # then tau_*
        k                               
    )
)

with open(OUT_CSV, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

    for row in combined_rows:
        out = {}
        for k in fieldnames:
            v = row.get(k)
            if isinstance(v, float):
                out[k] = f"{v:.16f}"   # full double precision
            else:
                out[k] = v
        writer.writerow(out)

print(f"[INFO] Wrote combined CSV to {OUT_CSV}")
