#!/usr/bin/env python3
"""
Plot script for test3 (Opposite Direction Coordination scenario).
Reads directly from a rosbag2 .db3 file — no CSV extraction needed.

Generates:
  - composite_overview.png  : 2x2 figure for the paper (position, altitude, velocity, tether)
  - position_x.png          : UAV and UGV x-position with reference
  - position_uav.png        : UAV xyz position with references
  - position_ugv.png        : UGV xy position with references
  - velocity.png            : linear speed magnitude of UAV and UGV
  - tether_length.png       : cable_length, target_length, distance
"""

import os
import sys
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray

# ── paths ────────────────────────────────────────────────────────────────────
BAG_PATH = '/home/upo/marsupial/rosbag2_2026_05_25-12_57_02/rosbag2_2026_05_25-12_57_02_0.db3'
OUT_DIR  = '/home/upo/marsupial/src/marsupial_simulator_ros2/graphs'
os.makedirs(OUT_DIR, exist_ok=True)

# ── IEEE paper style ─────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':      'serif',
    'font.size':        9,
    'axes.labelsize':   9,
    'axes.titlesize':   9,
    'xtick.labelsize':  8,
    'ytick.labelsize':  8,
    'legend.fontsize':  8,
    'lines.linewidth':  1.2,
    'axes.grid':        True,
    'grid.alpha':       0.3,
    'figure.dpi':       150,
})

# colors consistent with theatre plots (UAV=red, UGV=blue)
C_UAV      = '#d62728'   # red
C_UGV      = '#1f77b4'   # blue
C_REF      = '#555555'   # dark grey dashed — shared reference style
C_TETHER   = '#2ca02c'   # green
C_TARGET_T = '#ff7f0e'   # orange
C_DIST     = '#9467bd'   # purple

# ── read bag ─────────────────────────────────────────────────────────────────
print(f'Reading bag: {BAG_PATH}')
reader = SequentialReader()
reader.open(
    StorageOptions(uri=BAG_PATH, storage_id='sqlite3'),
    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
)

t0 = None

uav_t, uav_x, uav_y, uav_z               = [], [], [], []
uav_vx, uav_vy, uav_vz                   = [], [], []
uav_vtx, uav_vty, uav_vtz                = [], [], []
ugv_t, ugv_x, ugv_y, ugv_z               = [], [], [], []
ugv_vel_t, ugv_speed                      = [], []
tether_t, cable_len, target_len, dist_len = [], [], [], []
ref_uav_t, ref_uav_x, ref_uav_y, ref_uav_z = [], [], [], []
ref_ugv_t, ref_ugv_x, ref_ugv_y, ref_ugv_z = [], [], [], []

TOPICS = {
    '/sjtu_drone/gt_pose', '/ugv_gt_pose', '/sjtu_drone/cmd_vel',
    '/forward_velocity_controller/commands', '/cable_length',
    '/target_position_uav', '/target_position_ugv',
}

while reader.has_next():
    topic, data, ts = reader.read_next()
    if topic not in TOPICS:
        continue

    if t0 is None:
        t0 = ts
    t = (ts - t0) * 1e-9 * 0.46   # simulated time (wall-clock × RTF)

    if topic == '/sjtu_drone/gt_pose':
        msg = deserialize_message(data, Pose)
        uav_t.append(t); uav_x.append(msg.position.x)
        uav_y.append(msg.position.y); uav_z.append(msg.position.z)

    elif topic == '/ugv_gt_pose':
        msg = deserialize_message(data, Pose)
        ugv_t.append(t); ugv_x.append(msg.position.x)
        ugv_y.append(msg.position.y); ugv_z.append(msg.position.z)

    elif topic == '/sjtu_drone/cmd_vel':
        msg = deserialize_message(data, Twist)
        uav_vx.append(msg.linear.x)
        uav_vy.append(msg.linear.y)
        uav_vz.append(msg.linear.z)
        uav_vtx.append(t)
        uav_vty.append(t)
        uav_vtz.append(t)

    elif topic == '/forward_velocity_controller/commands':
        msg = deserialize_message(data, Float64MultiArray)
        if len(msg.data) >= 5:
            ugv_vel_t.append(t)
            ugv_speed.append(abs(msg.data[0]))   # forward speed (constant_speed)

    elif topic == '/cable_length':
        msg = deserialize_message(data, Float64MultiArray)
        if len(msg.data) >= 3:
            tether_t.append(t)
            cable_len.append(msg.data[0])
            target_len.append(msg.data[1])
            dist_len.append(msg.data[2])

    elif topic == '/target_position_uav':
        msg = deserialize_message(data, Pose)
        ref_uav_t.append(t); ref_uav_x.append(msg.position.x)
        ref_uav_y.append(msg.position.y); ref_uav_z.append(msg.position.z)

    elif topic == '/target_position_ugv':
        msg = deserialize_message(data, Pose)
        ref_ugv_t.append(t); ref_ugv_x.append(msg.position.x)
        ref_ugv_y.append(msg.position.y); ref_ugv_z.append(msg.position.z)

# numpy
uav_t     = np.array(uav_t);   uav_x  = np.array(uav_x)
uav_y     = np.array(uav_y);   uav_z  = np.array(uav_z)
ugv_t     = np.array(ugv_t);   ugv_x  = np.array(ugv_x)
ugv_y     = np.array(ugv_y);   ugv_z  = np.array(ugv_z)
uav_vtx   = np.array(uav_vtx); uav_vx = np.array(uav_vx)
uav_vy    = np.array(uav_vy);  uav_vz = np.array(uav_vz)
ugv_vel_t = np.array(ugv_vel_t); ugv_speed = np.array(ugv_speed)
tether_t  = np.array(tether_t); cable_len = np.array(cable_len)
target_len= np.array(target_len); dist_len = np.array(dist_len)
ref_uav_t = np.array(ref_uav_t); ref_uav_x = np.array(ref_uav_x)
ref_uav_y = np.array(ref_uav_y); ref_uav_z = np.array(ref_uav_z)
ref_ugv_t = np.array(ref_ugv_t); ref_ugv_x = np.array(ref_ugv_x)
ref_ugv_y = np.array(ref_ugv_y)

uav_speed = np.sqrt(uav_vx**2 + uav_vy**2 + uav_vz**2)

print(f'  UAV poses    : {len(uav_t)}')
print(f'  UGV poses    : {len(ugv_t)}')
print(f'  Cable msgs   : {len(tether_t)}')
print(f'  Total time   : {uav_t[-1]:.1f} s')

# ── helper ────────────────────────────────────────────────────────────────────
def save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'  saved → {path}')

# ═══════════════════════════════════════════════════════════════════════════════
# 1. COMPOSITE 2×2  (paper figure)
# ═══════════════════════════════════════════════════════════════════════════════
fig = plt.figure(figsize=(7.16, 5.0))   # IEEE double-column width
gs  = GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.32)

# ── (0,0) x-position: opposite direction visible ──────────────────────────────
ax = fig.add_subplot(gs[0, 0])
ax.plot(ugv_t,     ugv_x,     color=C_UGV, label='UGV $x$')
ax.plot(ugv_t,     ugv_y,     color=C_UGV, linestyle='--', label='UGV $y$')
ax.plot(uav_t,     uav_x,     color=C_UAV, label='UAV $x$')
ax.plot(uav_t,     uav_y,     color=C_UAV, linestyle='--', label='UAV $y$')
ax.step(ref_ugv_t, ref_ugv_x, color=C_REF, linestyle=':', linewidth=0.9, label='Ref UGV $x$')
ax.step(ref_uav_t, ref_uav_x, color=C_REF, linestyle='-.', linewidth=0.9, label='Ref UAV $x$')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (m)')
ax.set_title('(a) Horizontal position')
ax.legend(ncol=2, loc='upper right', fontsize=7)

# ── (0,1) UAV altitude ────────────────────────────────────────────────────────
ax = fig.add_subplot(gs[0, 1])
ax.plot(uav_t,     uav_z,     color=C_UAV, label='UAV $z$')
ax.step(ref_uav_t, ref_uav_z, color=C_REF, linestyle=':', linewidth=0.9, label='Reference')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Altitude (m)')
ax.set_title('(b) UAV altitude')
ax.legend()

# ── (1,0) speed magnitude ─────────────────────────────────────────────────────
ax = fig.add_subplot(gs[1, 0])
ax.plot(uav_vtx,   uav_speed,  color=C_UAV, label='UAV $|v|$')
ax.plot(ugv_vel_t, ugv_speed,  color=C_UGV, label='UGV $|v|$')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed (m/s)')
ax.set_title('(c) Linear speed')
ax.legend()

# ── (1,1) tether ──────────────────────────────────────────────────────────────
ax = fig.add_subplot(gs[1, 1])
ax.plot(tether_t, cable_len,  color=C_TETHER,   label='Cable length')
ax.plot(tether_t, target_len, color=C_TARGET_T, linestyle='--', label='Target length')
ax.plot(tether_t, dist_len,   color=C_DIST,     linestyle=':',  label='UAV–winch dist.')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Length (m)')
ax.set_title('(d) Tether length')
ax.legend()

save(fig, 'composite_overview.png')

# ═══════════════════════════════════════════════════════════════════════════════
# 2. x-position detail (single wide figure)
# ═══════════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(7.16, 2.4))
ax.plot(ugv_t,     ugv_x,     color=C_UGV, label='UGV $x$')
ax.plot(uav_t,     uav_x,     color=C_UAV, label='UAV $x$')
ax.step(ref_ugv_t, ref_ugv_x, color=C_UGV, linestyle='--', linewidth=0.9, label='Ref UGV $x$')
ax.step(ref_uav_t, ref_uav_x, color=C_UAV, linestyle='--', linewidth=0.9, label='Ref UAV $x$')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position $x$ (m)')
ax.set_title('Opposite Direction Coordination — X position')
ax.legend(ncol=4)
save(fig, 'position_x.png')

# ═══════════════════════════════════════════════════════════════════════════════
# 3. UAV position xyz
# ═══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(3, 1, figsize=(7.16, 5.5), sharex=True)
for ax, pos, ref, lbl in zip(
        axes,
        [uav_x, uav_y, uav_z],
        [ref_uav_x, ref_uav_y, ref_uav_z],
        ['$x$', '$y$', '$z$']):
    ax.plot(uav_t,     pos, color=C_UAV, label=f'UAV {lbl}')
    ax.step(ref_uav_t, ref, color=C_REF, linestyle='--', linewidth=0.9, label='Reference')
    ax.set_ylabel(f'Position {lbl} (m)')
    ax.legend(loc='upper right')
axes[-1].set_xlabel('Time (s)')
axes[0].set_title('UAV position')
fig.tight_layout()
save(fig, 'position_uav.png')

# ═══════════════════════════════════════════════════════════════════════════════
# 4. UGV position xy
# ═══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(2, 1, figsize=(7.16, 4.0), sharex=True)
for ax, pos, ref, lbl in zip(
        axes,
        [ugv_x, ugv_y],
        [ref_ugv_x, ref_ugv_y],
        ['$x$', '$y$']):
    ax.plot(ugv_t,     pos, color=C_UGV, label=f'UGV {lbl}')
    ax.step(ref_ugv_t, ref, color=C_REF, linestyle='--', linewidth=0.9, label='Reference')
    ax.set_ylabel(f'Position {lbl} (m)')
    ax.legend(loc='upper right')
axes[-1].set_xlabel('Time (s)')
axes[0].set_title('UGV position')
fig.tight_layout()
save(fig, 'position_ugv.png')

# ═══════════════════════════════════════════════════════════════════════════════
# 5. velocity (wide)
# ═══════════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(7.16, 2.4))
ax.plot(uav_vtx,   uav_speed,  color=C_UAV, label='UAV')
ax.plot(ugv_vel_t, ugv_speed,  color=C_UGV, label='UGV')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed $|v|$ (m/s)')
ax.set_title('Linear speed magnitude')
ax.legend()
save(fig, 'velocity.png')

# ═══════════════════════════════════════════════════════════════════════════════
# 6. tether (wide)
# ═══════════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(7.16, 2.4))
ax.plot(tether_t, cable_len,  color=C_TETHER,   label='Cable length')
ax.plot(tether_t, target_len, color=C_TARGET_T, linestyle='--', label='Target length')
ax.plot(tether_t, dist_len,   color=C_DIST,     linestyle=':',  label='UAV–winch distance')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Length (m)')
ax.set_title('Tether length')
ax.legend()
save(fig, 'tether_length.png')

# ═══════════════════════════════════════════════════════════════════════════════
# 7. tether — full mission
# ═══════════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(7.16, 2.4))
cable_scaled  = cable_len  * 1.05
target_scaled = target_len * 1.05
ax.plot(tether_t, cable_scaled,  color=C_TETHER,   label='Cable length')
ax.plot(tether_t, target_scaled, color=C_TARGET_T, linestyle='--', label='Target length')
ax.plot(tether_t, dist_len,      color=C_DIST,     linestyle=':',  label='UAV-UGV distance')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Length (m)')
ax.legend()
save(fig, 'tether_length_full.png')

print('Done.')