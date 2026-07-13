#!/usr/bin/env python3
"""
Single overview figure: position (left Y) + velocity (right Y), twin axes.
  Solid  = with tether
  Dashed = without tether   (same color per robot)
  Red    = UAV
  Blue   = UGV

Time is simulation time:
  t_sim = t_wall_clock × RTF
  RTF no-tether : 0.98
  RTF tether    : 0.46
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray

# ── paths ────────────────────────────────────────────────────────────────────
BAG_TETHER   = '/home/upo/marsupial/rosbag2_2026_05_25-12_57_02/rosbag2_2026_05_25-12_57_02_0.db3'
BAG_NOTETHER = '/home/upo/marsupial/rosbag2_2026_05_25-14_16_08/rosbag2_2026_05_25-14_16_08_0.db3'
RTF_TETHER   = 0.46
RTF_NOTETHER = 0.98
OUT_DIR      = '/home/upo/marsupial/src/marsupial_simulator_ros2/graphs'
os.makedirs(OUT_DIR, exist_ok=True)

# ── style ────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':    'serif',
    'font.size':      9,
    'axes.labelsize': 9,
    'axes.titlesize': 9,
    'xtick.labelsize':8,
    'ytick.labelsize':8,
    'legend.fontsize':7.5,
    'lines.linewidth':1.2,
    'axes.grid':      True,
    'grid.alpha':     0.3,
    'figure.dpi':     150,
})

C_UAV = '#d62728'   # red
C_UGV = '#1f77b4'   # blue

# ── read bag ─────────────────────────────────────────────────────────────────
TOPICS = {'/sjtu_drone/gt_pose', '/ugv_gt_pose',
          '/sjtu_drone/cmd_vel', '/forward_velocity_controller/commands'}

def read_bag(path, rtf):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='sqlite3'),
                ConverterOptions('cdr', 'cdr'))
    t0 = None
    uav_t, uav_x = [], []
    ugv_t, ugv_x = [], []
    uav_spd_t, uav_spd = [], []
    ugv_spd_t, ugv_spd = [], []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in TOPICS:
            continue
        if t0 is None:
            t0 = ts
        t = (ts - t0) * 1e-9 * rtf   # simulation time

        if topic == '/sjtu_drone/gt_pose':
            m = deserialize_message(data, Pose)
            uav_t.append(t); uav_x.append(m.position.x)

        elif topic == '/ugv_gt_pose':
            m = deserialize_message(data, Pose)
            ugv_t.append(t); ugv_x.append(m.position.x)

        elif topic == '/sjtu_drone/cmd_vel':
            m = deserialize_message(data, Twist)
            spd = (m.linear.x**2 + m.linear.y**2 + m.linear.z**2)**0.5
            uav_spd_t.append(t); uav_spd.append(spd)

        elif topic == '/forward_velocity_controller/commands':
            m = deserialize_message(data, Float64MultiArray)
            if len(m.data) >= 1:
                ugv_spd_t.append(t); ugv_spd.append(abs(m.data[0]))

    return {
        'uav_t':     np.array(uav_t),   'uav_x':  np.array(uav_x),
        'ugv_t':     np.array(ugv_t),   'ugv_x':  np.array(ugv_x),
        'uav_spd_t': np.array(uav_spd_t), 'uav_spd': np.array(uav_spd),
        'ugv_spd_t': np.array(ugv_spd_t), 'ugv_spd': np.array(ugv_spd),
    }

print('Reading tether bag   (RTF=0.46) …')
T = read_bag(BAG_TETHER,   RTF_TETHER)
print('Reading no-tether bag (RTF=0.98) …')
N = read_bag(BAG_NOTETHER, RTF_NOTETHER)

# ── figure ────────────────────────────────────────────────────────────────────
fig, ax_pos = plt.subplots(figsize=(7.16, 3.2))

LW = 1.2

ax_pos.plot(T['uav_t'], T['uav_x'], color=C_UAV, linestyle='-',  lw=LW, label='UAV $x$ (tether)')
ax_pos.plot(N['uav_t'], N['uav_x'], color=C_UAV, linestyle='--', lw=LW, label='UAV $x$ (no tether)')
ax_pos.plot(T['ugv_t'], T['ugv_x'], color=C_UGV, linestyle='-',  lw=LW, label='UGV $x$ (tether)')
ax_pos.plot(N['ugv_t'], N['ugv_x'], color=C_UGV, linestyle='--', lw=LW, label='UGV $x$ (no tether)')

ax_pos.set_xlabel('Time (s)')
ax_pos.set_ylabel('Position $x$ (m)')
ax_pos.legend(loc='upper left', frameon=True)

fig.tight_layout()

path = os.path.join(OUT_DIR, 'overview_twinax.png')
fig.savefig(path, dpi=300, bbox_inches='tight')
plt.close(fig)
print(f'saved → {path}')
print('Done.')
