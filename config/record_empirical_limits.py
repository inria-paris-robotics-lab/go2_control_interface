#!/usr/bin/env python3
"""
Record empirical joint position limits for the G1 robot.

Usage:
    python3 record_empirical_limits.py

    1. Move the robot joints to their extreme positions.
    2. The script continuously records the observed min/max and writes
       g1_custom_limits.yaml in this directory.
    3. Press Ctrl+C to exit (the file is saved one last time on exit).

If g1_custom_limits.yaml already exists, existing bounds are preserved and
only extended as new extremes are observed (session is resumable).

The recorder is authored for the full 29-DOF G1: it samples all 29 hardware
joints (waist_roll/waist_pitch included). On a 27-DOF robot (mode 6) the two
waist joints are mechanically locked, so they simply never move and keep the
default bound on save; the 27-DOF watchdog ignores indices 13/14 anyway.
"""

import math
import os
import signal
import sys

import rclpy
import yaml
from rclpy.node import Node
from unitree_hg.msg import LowState

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(_HERE, "g1_custom_limits.yaml")
DEFAULT_FILE = os.path.join(_HERE, "g1_default_limits.yaml")

# ---------------------------------------------------------------------------
# G1 constants  (from g1_interface.py, 29-DOF set)
#
# Array index == unitree hardware index 0..28. waist_roll(13)/waist_pitch(14)
# are actuated only in 29-DOF (mode 5); on a 27-DOF robot they stay locked.
# ---------------------------------------------------------------------------
URDF_TO_UNITREE = tuple(range(29))
N_DOF = 29

JOINT_NAMES = [
    # left leg
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee",
    "L_ankle_pitch", "L_ankle_roll",
    # right leg
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee",
    "R_ankle_pitch", "R_ankle_roll",
    # waist (3 joints; roll/pitch locked in 27-DOF)
    "waist_yaw", "waist_roll", "waist_pitch",
    # left arm
    "L_sho_pitch", "L_sho_roll", "L_sho_yaw", "L_elbow",
    "L_wrist_roll", "L_wrist_pitch", "L_wrist_yaw",
    # right arm
    "R_sho_pitch", "R_sho_roll", "R_sho_yaw", "R_elbow",
    "R_wrist_roll", "R_wrist_pitch", "R_wrist_yaw",
]

# Margin durations copied from default limits (end-of-chain joints use 0.01)
# Capture frequency (Hz), how often joint positions are sampled (G1 lowstate is 1000 Hz max)
CAPTURE_FREQ_HZ = 1000.0
# Refresh / save frequency (Hz), how often the display and YAML file are updated
DISPLAY_FREQ_HZ = 10.0

DEFAULT_MARGIN_DURATION = [
    0.03, 0.03, 0.03, 0.03, 0.03, 0.03,  # left leg
    0.03, 0.03, 0.03, 0.03, 0.03, 0.03,  # right leg
    0.03, 0.03, 0.03,                     # waist (yaw, roll, pitch)
    0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03,  # left arm
    0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03,  # right arm
]


# ---------------------------------------------------------------------------
# YAML helpers
# ---------------------------------------------------------------------------

def _load_default_limits():
    with open(DEFAULT_FILE) as f:
        data = yaml.safe_load(f)
    params = data["watchdog"]["ros__parameters"]
    return list(params["q_min"]), list(params["q_max"])


def _load_existing_limits():
    """Return (q_min, q_max) from an existing custom file, or None if absent."""
    if not os.path.exists(OUTPUT_FILE):
        return None, None
    with open(OUTPUT_FILE) as f:
        data = yaml.safe_load(f)
    params = data["watchdog"]["ros__parameters"]
    return list(params["q_min"]), list(params["q_max"])


def _save_limits(q_min, q_max, default_q_min, default_q_max):
    """Write g1_custom_limits.yaml.

    For joints not yet observed (still at sentinel ±inf), fall back to the
    default limit so the file is always safe to load by the watchdog.
    """
    saved_min = [
        round(default_q_min[i], 6) if math.isinf(q_min[i]) else round(q_min[i], 6)
        for i in range(N_DOF)
    ]
    saved_max = [
        round(default_q_max[i], 6) if math.isinf(q_max[i]) else round(q_max[i], 6)
        for i in range(N_DOF)
    ]
    data = {
        "watchdog": {
            "ros__parameters": {
                "q_max": saved_max,
                "q_min": saved_min,
                "margin_duration": DEFAULT_MARGIN_DURATION,
            }
        }
    }
    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)


# ---------------------------------------------------------------------------
# Display helper
# ---------------------------------------------------------------------------

def _display(q_min, q_max, default_q_min, default_q_max, n_samples):
    # Move cursor to top of screen instead of clearing (avoids flicker)
    print("\033[H", end="")
    print(f"=== G1 Empirical Limits Recorder  —  {n_samples} samples ===\n")
    print(f"  {'Joint':<18} {'obs_min':>9} {'obs_max':>9}  "
          f"{'def_min':>9} {'def_max':>9}  {'explored':>8}")
    print("  " + "-" * 68)
    for i in range(N_DOF):
        has_data = not (math.isinf(q_min[i]) or math.isinf(q_max[i]))
        if has_data:
            obs_min_s = f"{q_min[i]:+.4f}"
            obs_max_s = f"{q_max[i]:+.4f}"
            explored = f"{q_max[i] - q_min[i]:.4f} rad"
        else:
            obs_min_s = "   ---"
            obs_max_s = "   ---"
            explored = "     ---"
        def_min_s = f"{default_q_min[i]:+.4f}"
        def_max_s = f"{default_q_max[i]:+.4f}"
        print(f"  {JOINT_NAMES[i]:<18} {obs_min_s:>9} {obs_max_s:>9}  "
              f"{def_min_s:>9} {def_max_s:>9}  {explored:>8}")
    n_observed = sum(1 for i in range(N_DOF) if not math.isinf(q_min[i]))
    print(f"\n  {n_observed}/{N_DOF} joints observed  —  saving to: {OUTPUT_FILE}")
    print("  Press Ctrl+C to stop.\n")


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class EmpiricalLimitsRecorder(Node):
    def __init__(self, q_min, q_max, default_q_min, default_q_max):
        super().__init__("empirical_limits_recorder")

        self.q_min = q_min
        self.q_max = q_max
        self.default_q_min = default_q_min
        self.default_q_max = default_q_max
        self.n_samples = 0
        self._latest_msg = None

        self._sub = self.create_subscription(LowState, "lowstate", self._state_cb, 10)
        self._capture_timer = self.create_timer(1.0 / CAPTURE_FREQ_HZ, self._capture)
        self._display_timer = self.create_timer(1.0 / DISPLAY_FREQ_HZ, self._tick)

        print("\033[2J", end="")  # clear screen once at start

    def _state_cb(self, msg):
        # Just buffer the latest message — processing is done at CAPTURE_FREQ_HZ
        self._latest_msg = msg

    def _capture(self):
        if self._latest_msg is None:
            return
        msg = self._latest_msg
        for i_urdf, i_hw in enumerate(URDF_TO_UNITREE):
            q = msg.motor_state[i_hw].q
            if q < self.q_min[i_urdf]:
                self.q_min[i_urdf] = q
            if q > self.q_max[i_urdf]:
                self.q_max[i_urdf] = q
        self.n_samples += 1

    def _tick(self):
        _save_limits(self.q_min, self.q_max, self.default_q_min, self.default_q_max)
        _display(self.q_min, self.q_max, self.default_q_min, self.default_q_max, self.n_samples)

    def shutdown_save(self):
        _save_limits(self.q_min, self.q_max, self.default_q_min, self.default_q_max)
        print(f"\nLimits saved to {OUTPUT_FILE}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    default_q_min, default_q_max = _load_default_limits()

    existing_q_min, existing_q_max = _load_existing_limits()
    if existing_q_min is not None:
        q_min = existing_q_min
        q_max = existing_q_max
        print(f"Resumed from existing {OUTPUT_FILE}")
    else:
        q_min = [math.inf] * N_DOF
        q_max = [-math.inf] * N_DOF
        print("Starting fresh (no existing custom limits file).")

    rclpy.init()
    node = EmpiricalLimitsRecorder(q_min, q_max, default_q_min, default_q_max)

    def _on_sigint(sig, frame):
        node.shutdown_save()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_sigint)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
