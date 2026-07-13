#!/usr/bin/env python3
"""
Comparison plot: with tether vs without tether (test3 scenario).

Time synchronization:
  - No-tether bag is recorded at RTF≈1 → wall-clock time ≈ simulation time.
  - Tether bag wall-clock time is scaled by  T_notether / T_tether  so both
    missions start and end at the same simulated time.
  - The implied RTF of the tether simulation is that same scale factor.

Outputs (in graphs/):
  - comparison_composite.png   : 2×3 grid — UAV xyz + UGV xy + speed  (paper figure)
  - comparison_uav_x.png / _y / _z
  - comparison_ugv_x.png / _y
  - comparison_velocity.png
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
    'legend.fontsize':8,
    'lines.linewidth':1.1,
    'axes.grid':      True,
    'grid.alpha':     0.3,
    'figure.dpi':     150,
})

C_TETHER   = '#d62728'   # red  — with tether
C_NOTETHER = '#1f77b4'   # blue — without tether
C_REF      = '#555555'   # grey dashed — reference

# ── read one bag ─────────────────────────────────────────────────────────────
TOPICS = {'/sjtu_drone/gt_pose', '/ugv_gt_pose',
          '/sjtu_drone/cmd_vel', '/forward_velocity_controller/commands'}

def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='sqlite3'),
                ConverterOptions('cdr', 'cdr'))
    t0 = None
    uav_t, uav_x, uav_y, uav_z = [], [], [], []
    ugv_t, ugv_x, ugv_y        = [], [], []
    uav_spd_t, uav_spd         = [], []
    ugv_spd_t, ugv_spd         = [], []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in TOPICS:
            continue
        if t0 is None:
            t0 = ts
        t = (ts - t0) * 1e-9

        if topic == '/sjtu_drone/gt_pose':
            m = deserialize_message(data, Pose)
            uav_t.append(t); uav_x.append(m.position.x)
            uav_y.append(m.position.y); uav_z.append(m.position.z)

        elif topic == '/ugv_gt_pose':
            m = deserialize_message(data, Pose)
            ugv_t.append(t); ugv_x.append(m.position.x); ugv_y.append(m.position.y)

        elif topic == '/sjtu_drone/cmd_vel':
            m = deserialize_message(data, Twist)
            spd = (m.linear.x**2 + m.linear.y**2 + m.linear.z**2)**0.5
            uav_spd_t.append(t); uav_spd.append(spd)

        elif topic == '/forward_velocity_controller/commands':
            m = deserialize_message(data, Float64MultiArray)
            if len(m.data) >= 1:
                ugv_spd_t.append(t); ugv_spd.append(abs(m.data[0]))

    return {
        'uav_t':     np.array(uav_t),
        'uav_x':     np.array(uav_x),
        'uav_y':     np.array(uav_y),
        'uav_z':     np.array(uav_z),
        'ugv_t':     np.array(ugv_t),
        'ugv_x':     np.array(ugv_x),
        'ugv_y':     np.array(ugv_y),
        'uav_spd_t': np.array(uav_spd_t),
        'uav_spd':   np.array(uav_spd),
        'ugv_spd_t': np.array(ugv_spd_t),
        'ugv_spd':   np.array(ugv_spd),
        'T':         (ts - t0) * 1e-9,   # total wall-clock duration
    }

# ── load ─────────────────────────────────────────────────────────────────────
print('Reading tether bag …')
T = read_bag(BAG_TETHER)
print('Reading no-tether bag …')
N = read_bag(BAG_NOTETHER)

# ── time synchronization ──────────────────────────────────────────────────────
# No-tether RTF≈1 → its wall-clock ≈ simulation time.
# Scale tether wall-clock so both missions span the same simulation duration.
scale = N['T'] / T['T']
implied_rtf = scale

print()
print('══════════════════════════════════════════')
print(f'  No-tether wall-clock duration : {N["T"]:.1f} s  ({N["T"]/60:.2f} min)')
print(f'  Tether   wall-clock duration  : {T["T"]:.1f} s  ({T["T"]/60:.2f} min)')
print(f'  Time scale factor (RTF)       : {scale:.4f}')
print(f'  Implied tether RTF            : {implied_rtf:.4f}  (~{implied_rtf:.2f})')
print('══════════════════════════════════════════')
print()

# Apply scale to tether time arrays
for key in ('uav_t', 'ugv_t', 'uav_spd_t', 'ugv_spd_t'):
    T[key] = T[key] * scale

# ── helper ────────────────────────────────────────────────────────────────────
def save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'  saved → {path}')

def ax_panel(ax, t_t, y_t, t_n, y_n, ylabel, title):
    ax.plot(t_n, y_n, color=C_NOTETHER, label='Without tether', linewidth=1.0)
    ax.plot(t_t, y_t, color=C_TETHER,   label='With tether',    linewidth=1.0, alpha=0.85)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc='upper right')

# ═══════════════════════════════════════════════════════════════════════════════
# 1.  COMPOSITE 2×3  (paper figure)
# ═══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(2, 3, figsize=(7.16, 4.8), sharex=True)
fig.subplots_adjust(hspace=0.42, wspace=0.38)

ax_panel(axes[0,0], T['uav_t'], T['uav_x'], N['uav_t'], N['uav_x'], 'Position (m)', '(a) UAV $x$')
ax_panel(axes[0,1], T['uav_t'], T['uav_y'], N['uav_t'], N['uav_y'], 'Position (m)', '(b) UAV $y$')
ax_panel(axes[0,2], T['uav_t'], T['uav_z'], N['uav_t'], N['uav_z'], 'Altitude (m)', '(c) UAV $z$')
ax_panel(axes[1,0], T['ugv_t'], T['ugv_x'], N['ugv_t'], N['ugv_x'], 'Position (m)', '(d) UGV $x$')
ax_panel(axes[1,1], T['ugv_t'], T['ugv_y'], N['ugv_t'], N['ugv_y'], 'Position (m)', '(e) UGV $y$')

# panel (f): speed comparison
axes[1,2].plot(N['uav_spd_t'], N['uav_spd'], color=C_NOTETHER, linestyle='-',  label='UAV (no tether)', linewidth=1.0)
axes[1,2].plot(T['uav_spd_t'], T['uav_spd'], color=C_TETHER,   linestyle='-',  label='UAV (tether)',    linewidth=1.0, alpha=0.85)
axes[1,2].plot(N['ugv_spd_t'], N['ugv_spd'], color=C_NOTETHER, linestyle='--', label='UGV (no tether)', linewidth=1.0)
axes[1,2].plot(T['ugv_spd_t'], T['ugv_spd'], color=C_TETHER,   linestyle='--', label='UGV (tether)',    linewidth=1.0, alpha=0.85)
axes[1,2].set_ylabel('Speed (m/s)')
axes[1,2].set_title('(f) Linear speed $|v|$')
axes[1,2].legend(fontsize=6, ncol=2)

for ax in axes[1]:
    ax.set_xlabel('Time (s)')

save(fig, 'comparison_composite.png')

# ═══════════════════════════════════════════════════════════════════════════════
# 2.  Individual axis figures
# ═══════════════════════════════════════════════════════════════════════════════
for label, key_t, key_n, ylabel, fname in [
    ('UAV $x$ position',  'uav_x', 'uav_x', 'Position (m)', 'comparison_uav_x.png'),
    ('UAV $y$ position',  'uav_y', 'uav_y', 'Position (m)', 'comparison_uav_y.png'),
    ('UAV $z$ altitude',  'uav_z', 'uav_z', 'Altitude (m)', 'comparison_uav_z.png'),
    ('UGV $x$ position',  'ugv_x', 'ugv_x', 'Position (m)', 'comparison_ugv_x.png'),
    ('UGV $y$ position',  'ugv_y', 'ugv_y', 'Position (m)', 'comparison_ugv_y.png'),
]:
    t_key = 'uav_t' if 'uav' in fname else 'ugv_t'
    fig, ax = plt.subplots(figsize=(7.16, 2.4))
    ax.plot(N[t_key], N[key_n], color=C_NOTETHER, label='Without tether')
    ax.plot(T[t_key], T[key_t], color=C_TETHER,   label='With tether', alpha=0.85)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(label)
    ax.legend()
    save(fig, fname)

# ═══════════════════════════════════════════════════════════════════════════════
# 3.  Velocity comparison
# ═══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(2, 1, figsize=(7.16, 4.0), sharex=True)
axes[0].plot(N['uav_spd_t'], N['uav_spd'], color=C_NOTETHER, label='Without tether')
axes[0].plot(T['uav_spd_t'], T['uav_spd'], color=C_TETHER,   label='With tether', alpha=0.85)
axes[0].set_ylabel('Speed (m/s)'); axes[0].set_title('UAV speed $|v|$'); axes[0].legend()

axes[1].plot(N['ugv_spd_t'], N['ugv_spd'], color=C_NOTETHER, label='Without tether')
axes[1].plot(T['ugv_spd_t'], T['ugv_spd'], color=C_TETHER,   label='With tether', alpha=0.85)
axes[1].set_ylabel('Speed (m/s)'); axes[1].set_title('UGV speed $|v|$')
axes[1].set_xlabel('Time (s)'); axes[1].legend()

fig.tight_layout()
save(fig, 'comparison_velocity.png')

print('Done.')
