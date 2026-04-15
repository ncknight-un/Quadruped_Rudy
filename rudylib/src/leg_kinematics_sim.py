"""
Quadruped Leg Kinematics Simulator
===================================
Port of rudylib/LegKinematics.hpp to Python.

Coordinate convention:
  - Z up
  - Y forwards
  - X into the leg (lateral)
  - Link 1 (coxa) rotates about Y-axis (abduction/adduction)
  - Links 2-3 (femur, tibia) rotate about X-axis (hip, knee)

Walking waypoints are expressed as (abad, hip, knee) joint angles in radians.
The gait path is interpolated smoothly between waypoints and displayed as a 3-D
foot trajectory together with a live-updating stick figure of the leg.

Usage:
    python leg_kinematics_sim.py

Dependencies:
    pip install numpy matplotlib

Source: https://claude.ai/share/9cb5ff89-9d32-4ea0-896f-fa7d2c779131
This simulation was generated using claude from the Kinematics package that I wrote.
"""

import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401 (registers 3-D projection)
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from dataclasses import dataclass
from typing import Optional, List, Tuple


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class JointAngles:
    abad: float = 0.0   # abduction / adduction  (Y-axis rotation)
    hip:  float = 0.0   # hip flexion / extension (X-axis rotation)
    knee: float = 0.0   # knee flexion / extension (X-axis rotation)


# ---------------------------------------------------------------------------
# Core kinematics  (mirrors LegKinematics.hpp exactly)
# ---------------------------------------------------------------------------

class QuadrupedLeg:
    """
    3-DOF leg kinematics for a single quadruped leg.

    Parameters
    ----------
    coxa, femur, tibia : float
        Link lengths in the same unit (e.g. mm).
    legtype : str
        'L' for left leg, 'R' for right leg.
        Right-leg angles are sign-flipped on abad and knee to maintain a
        consistent positive-direction convention.
    abad_limits, hip_limits, knee_limits : (float, float)
        (min, max) joint limits in radians.  Set to (-pi, pi) to disable.
    """

    def __init__(
        self,
        coxa:  float = 23.0,
        femur: float = 77.0,
        tibia: float = 100.0,
        legtype: str = 'L',
        abad_limits:  Tuple[float, float] = (-math.pi, math.pi),
        hip_limits:   Tuple[float, float] = (-math.pi, math.pi),
        knee_limits:  Tuple[float, float] = (-math.pi, math.pi),
    ):
        self.coxa  = coxa
        self.femur = femur
        self.tibia = tibia
        self.legtype = legtype
        self.abad_limits  = abad_limits
        self.hip_limits   = hip_limits
        self.knee_limits  = knee_limits

    # ------------------------------------------------------------------
    # Domain check
    # ------------------------------------------------------------------

    def is_within_domain(self, target: np.ndarray) -> bool:
        """Return True if *target* (x, y, z) is reachable."""
        tx, ty, tz = target

        # Global reach
        L = math.sqrt(tx*tx + ty*ty + tz*tz)
        M = self.coxa + self.femur + self.tibia
        if L > M:
            return False

        # Lower-leg reach after coxa offset
        phi_abad = math.atan2(tx, tz)
        x_eff = self.coxa * math.sin(phi_abad)
        z_eff = self.coxa * math.cos(phi_abad)
        K = math.sqrt((tx - x_eff)**2 + ty**2 + (tz - z_eff)**2)

        if K > (self.femur + self.tibia) or K < abs(self.femur - self.tibia):
            return False

        return True

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def update_FK(self, angles: JointAngles) -> np.ndarray:
        """
        Convert joint angles → foot position in 3-D space.

        Returns
        -------
        np.ndarray  shape (3,)   [x, y, z]
        """
        abad, hip, knee = angles.abad, angles.hip, angles.knee

        x_hip = self.coxa * math.sin(abad)
        z_hip = self.coxa * math.cos(abad)

        r = (self.femur * math.sin(hip)
             + self.tibia * math.sin(hip + knee))
        h = (self.femur * math.cos(hip)
             + self.tibia * math.cos(hip + knee))

        x = x_hip + h * math.sin(abad)
        y = r
        z = -(z_hip + h * math.cos(abad))

        return np.array([x, y, z])

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------

    def calc_IK(self, target: np.ndarray) -> Optional[JointAngles]:
        """
        Convert foot position → joint angles.

        Returns None if the target is outside the reachable domain.
        """
        tx, ty, tz = target

        # Abduction angle
        abad = math.atan2(tx, -tz)

        # Hip-joint origin after coxa offset
        x_eff = self.coxa * math.sin(abad)
        z_eff = self.coxa * math.cos(abad)

        dx = tx - x_eff
        dy = ty
        dz = -tz - z_eff
        K  = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Domain guard
        if not self.is_within_domain(target):
            return None

        # Knee angle (law of cosines)
        cos_knee = ((self.femur**2 + self.tibia**2 - K**2)
                    / (2 * self.femur * self.tibia))
        cos_knee = max(-1.0, min(1.0, cos_knee))
        knee = _normalize_angle(math.pi + math.acos(cos_knee))

        # Hip angle
        alpha1  = math.atan2(dy, math.sqrt(dx*dx + dz*dz))
        cos_a2  = ((self.femur**2 + K**2 - self.tibia**2)
                   / (2 * self.femur * K))
        cos_a2  = max(-1.0, min(1.0, cos_a2))
        hip     = alpha1 + math.acos(cos_a2)

        # Right-leg sign flip
        if self.legtype == 'R':
            abad = -abad
            knee = -knee

        return JointAngles(abad=abad, hip=hip, knee=knee)

    # ------------------------------------------------------------------
    # Joint positions (for visualisation)
    # ------------------------------------------------------------------

    def joint_positions(self, angles: JointAngles) -> List[np.ndarray]:
        """
        Return the 3-D world positions of all joints + foot tip.

        Returns
        -------
        [origin, coxa_end (hip joint), femur_end (knee joint), foot]
        """
        abad, hip, knee = angles.abad, angles.hip, angles.knee

        origin = np.zeros(3)

        # End of coxa  (hip joint)
        x_hip = self.coxa * math.sin(abad)
        z_hip = self.coxa * math.cos(abad)
        p_hip = np.array([x_hip, 0.0, -z_hip])

        # End of femur  (knee joint)
        r1 = self.femur * math.sin(hip)
        h1 = self.femur * math.cos(hip)
        p_knee = np.array([
            x_hip + h1 * math.sin(abad),
            r1,
            -(z_hip + h1 * math.cos(abad)),
        ])

        # Foot tip
        foot = self.update_FK(angles)

        return [origin, p_hip, p_knee, foot]


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ---------------------------------------------------------------------------
# Walking gait waypoints
# ---------------------------------------------------------------------------

# Waypoints as (abad_rad, hip_rad, knee_rad)
# Source: walking_pose_sequence from config
WALKING_WAYPOINTS_RAD: List[Tuple[float, float, float]] = [
    # STANCE
    (0.0,  0.9,   -1.5),
    # LIFT UP
    (0.0, 0.9,   -1.7),
    # THRUST (push back)
    (0.0,  0.55,  -1.5),
    # LAND and Swing Forwards
    (0.0,  0.85,  -1.5),
]

WAYPOINT_LABELS = ["Stance", "Lift up", "Thrust", "Land / swing"]


def interpolate_gait(
    waypoints: List[Tuple[float, float, float]],
    steps_per_segment: int = 40,
    loop: bool = True,
) -> List[JointAngles]:
    """
    Linearly interpolate between waypoints.

    Parameters
    ----------
    waypoints : list of (abad, hip, knee) tuples in radians.
    steps_per_segment : number of intermediate frames between each pair.
    loop : if True, append transition back to the first waypoint.
    """
    pts = list(waypoints)
    if loop:
        pts.append(waypoints[0])

    frames: List[JointAngles] = []
    for i in range(len(pts) - 1):
        a0 = np.array(pts[i])
        a1 = np.array(pts[i + 1])
        for t in np.linspace(0, 1, steps_per_segment, endpoint=False):
            interp = a0 + t * (a1 - a0)
            frames.append(JointAngles(abad=interp[0], hip=interp[1], knee=interp[2]))

    return frames


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

LINK_COLORS  = ["#185FA5", "#BA7517", "#993C1D"]   # coxa, femur, tibia
JOINT_COLOR  = "#0F6E56"
FOOT_COLOR   = "#993556"
PATH_COLOR   = "#3B8BD4"
WP_COLOR     = "#D85A30"
BG_COLOR     = "#F1EFE8"
DARK_BG      = "#2C2C2A"


def build_figure(leg: QuadrupedLeg, frames: List[JointAngles]):
    """
    Build a matplotlib figure with:
      - 3-D animated stick figure of the leg
      - Full foot-path trajectory
      - Waypoint markers
      - Angle time-series plot (bottom)
    """

    # Pre-compute foot path and joint positions for all frames
    all_joints = [leg.joint_positions(f) for f in frames]
    all_feet   = np.array([jp[3] for jp in all_joints])
    wp_feet    = np.array([leg.update_FK(JointAngles(*w)) for w in WALKING_WAYPOINTS_RAD])

    fig = plt.figure(figsize=(14, 8), facecolor=BG_COLOR)
    fig.suptitle("Quadruped Leg Kinematics — Walking Gait", fontsize=13,
                 fontweight="bold", color="#2C2C2A", y=0.97)

    # ---- Layout ----
    gs = fig.add_gridspec(2, 2,
                          left=0.05, right=0.97,
                          top=0.92, bottom=0.08,
                          hspace=0.38, wspace=0.32)

    ax3d   = fig.add_subplot(gs[0, 0], projection='3d')
    ax_top = fig.add_subplot(gs[0, 1])
    ax_ang = fig.add_subplot(gs[1, :])

    # ------------------------------------------------------------------ 3-D view
    ax3d.set_facecolor(BG_COLOR)
    ax3d.set_xlabel("X (lateral)", fontsize=8, color="#444441")
    ax3d.set_ylabel("Y (forward)", fontsize=8, color="#444441")
    ax3d.set_zlabel("Z (up)",      fontsize=8, color="#444441")
    ax3d.set_title("3-D leg  +  foot path", fontsize=9, color="#444441", pad=4)
    ax3d.tick_params(labelsize=7)

    # Foot path ghost
    ax3d.plot(all_feet[:, 0], all_feet[:, 1], all_feet[:, 2],
              color=PATH_COLOR, alpha=0.35, linewidth=1.2, zorder=1)
    ax3d.scatter(*wp_feet.T, color=WP_COLOR, s=30, zorder=5, label="waypoints")
    for i, (wp, lbl) in enumerate(zip(wp_feet, WAYPOINT_LABELS)):
        ax3d.text(wp[0], wp[1], wp[2] + 4, lbl, fontsize=6, color=WP_COLOR)

    # Live stick-figure artists
    stick_lines3d = []
    for k in range(3):
        ln, = ax3d.plot([], [], [], color=LINK_COLORS[k], linewidth=3.5 - k*0.5,
                        solid_capstyle='round', zorder=3)
        stick_lines3d.append(ln)
    joint_scat3d = ax3d.scatter([], [], [], color=JOINT_COLOR, s=22, zorder=4)
    foot_scat3d  = ax3d.scatter([], [], [], color=FOOT_COLOR,  s=50, zorder=6,
                                marker='*')

    # Axis limits
    pad = 20
    mn = all_feet.min(axis=0) - pad
    mx = all_feet.max(axis=0) + pad
    center = (mn + mx) / 2
    half   = max((mx - mn).max() / 2, 80)
    ax3d.set_xlim(center[0]-half, center[0]+half)
    ax3d.set_ylim(center[1]-half, center[1]+half)
    ax3d.set_zlim(center[2]-half, center[2]+half)
    # ax3d.view_init(elev=20, azim=-55)
    ax3d.view_init(elev=20, azim=90)

    # ------------------------------------------------------------------ Side view (Y-Z)
    ax_top.set_facecolor(BG_COLOR)
    ax_top.set_aspect('equal')
    ax_top.set_xlabel("Y (forward)  [mm]", fontsize=8, color="#444441")
    ax_top.set_ylabel("Z (up)       [mm]", fontsize=8, color="#444441")
    ax_top.set_title("Side view  (Y – Z)", fontsize=9, color="#444441", pad=4)
    ax_top.tick_params(labelsize=7)
    ax_top.grid(True, linewidth=0.4, color="#B4B2A9", alpha=0.5)

    # Side view projection (Y-Z)
    ax_top.plot(all_feet[:, 1], all_feet[:, 2], color=PATH_COLOR, alpha=0.4, linewidth=1.2)

    ax_top.scatter(wp_feet[:, 1], wp_feet[:, 2], color=WP_COLOR, s=30, zorder=5)

    for wp, lbl in zip(wp_feet, WAYPOINT_LABELS):
        ax_top.text(wp[1] + 1, wp[2] + 1, lbl, fontsize=6, color=WP_COLOR)

    top_lines = []
    for k in range(3):
        ln, = ax_top.plot([], [], color=LINK_COLORS[k], linewidth=3 - k*0.4,
                          solid_capstyle='round')
        top_lines.append(ln)
    top_joints, = ax_top.plot([], [], 'o', color=JOINT_COLOR, ms=5, zorder=4)
    top_foot,   = ax_top.plot([], [], '*', color=FOOT_COLOR,   ms=8, zorder=5)

    # ------------------------------------------------------------------ Angle plot
    n = len(frames)
    ts = np.arange(n)
    abads = np.degrees([f.abad for f in frames])
    hips  = np.degrees([f.hip  for f in frames])
    knees = np.degrees([f.knee for f in frames])

    ax_ang.set_facecolor(BG_COLOR)
    ax_ang.set_xlim(0, n - 1)
    ax_ang.set_xlabel("Frame", fontsize=8, color="#444441")
    ax_ang.set_ylabel("Angle  (°)", fontsize=8, color="#444441")
    ax_ang.set_title("Joint angles over gait cycle", fontsize=9, color="#444441", pad=4)
    ax_ang.tick_params(labelsize=7)
    ax_ang.grid(True, linewidth=0.4, color="#B4B2A9", alpha=0.5)

    ax_ang.plot(ts, abads, color=LINK_COLORS[0], label="Abad",  linewidth=1.5)
    ax_ang.plot(ts, hips,  color=LINK_COLORS[1], label="Hip",   linewidth=1.5)
    ax_ang.plot(ts, knees, color=LINK_COLORS[2], label="Knee",  linewidth=1.5)

    # Waypoint vertical lines
    wp_frames = [i * (n // len(WALKING_WAYPOINTS_RAD)) for i in range(len(WALKING_WAYPOINTS_RAD))]
    for wf, lbl in zip(wp_frames, WAYPOINT_LABELS):
        ax_ang.axvline(wf, color=WP_COLOR, alpha=0.5, linewidth=0.8, linestyle='--')
        ax_ang.text(wf+0.5, ax_ang.get_ylim()[0] + 2, lbl,
                    fontsize=6, color=WP_COLOR, rotation=90, va='bottom')

    cursor_line = ax_ang.axvline(0, color="#888780", linewidth=1, linestyle=':')
    ax_ang.legend(fontsize=8, loc='upper right',
                  framealpha=0.7, edgecolor="#B4B2A9")

    # ------------------------------------------------------------------ Animation
    def update(frame_idx):
        joints = all_joints[frame_idx]   # list of 4 np.ndarray

        # 3-D stick
        for k, ln in enumerate(stick_lines3d):
            p0, p1 = joints[k], joints[k+1]
            ln.set_data([p0[0], p1[0]], [p0[1], p1[1]])
            ln.set_3d_properties([p0[2], p1[2]])

        jxyz = np.array(joints[:3])   # origin + hip + knee (not foot)
        joint_scat3d._offsets3d = (jxyz[:, 0], jxyz[:, 1], jxyz[:, 2])
        ft = joints[3]
        foot_scat3d._offsets3d  = ([ft[0]], [ft[1]], [ft[2]])

        # Side view (Y-Z)
        for k, ln in enumerate(top_lines):
            p0, p1 = joints[k], joints[k+1]
            ln.set_data([p0[1], p1[1]], [p0[2], p1[2]])

        jyz = np.array(joints[:3])
        top_joints.set_data(jyz[:, 1], jyz[:, 2])
        top_foot.set_data([ft[1]], [ft[2]])

        # Cursor
        cursor_line.set_xdata([frame_idx, frame_idx])

        return stick_lines3d + [joint_scat3d, foot_scat3d] + top_lines + \
               [top_joints, top_foot, cursor_line]

    ani = animation.FuncAnimation(
        fig, update,
        frames=len(frames),
        interval=30,
        blit=False,
    )

    return fig, ani


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # Leg configuration  — adjust link lengths and limits to match your robot
    leg = QuadrupedLeg(
        coxa  = 23.0,
        femur = 77.0,
        tibia = 100.0,
        legtype = 'R',
        abad_limits  = (-math.radians(45), math.radians(45)),
        hip_limits   = (-math.radians(90), math.radians(90)),
        knee_limits  = (-2*math.pi, 2*math.pi),
    )

    # Quick sanity check — FK then IK round-trip
    print("=== FK / IK round-trip sanity check ===")
    for i, (a, h, k) in enumerate(WALKING_WAYPOINTS_RAD):
        angles = JointAngles(abad=a, hip=h, knee=k)
        foot   = leg.update_FK(angles)
        ik_sol = leg.calc_IK(foot)
        ok     = "✓" if ik_sol is not None else "✗ (out of domain)"
        print(f"  WP{i+1} [{WAYPOINT_LABELS[i]:20s}]  "
              f"foot=({foot[0]:+7.2f}, {foot[1]:+7.2f}, {foot[2]:+7.2f}) mm  "
              f"IK: {ok}")
    print()

    # Build gait frames
    frames = interpolate_gait(WALKING_WAYPOINTS_RAD, steps_per_segment=50, loop=True)
    print(f"Gait cycle: {len(WALKING_WAYPOINTS_RAD)} waypoints → {len(frames)} frames\n")

    # Show
    fig, ani = build_figure(leg, frames)
    plt.show()


if __name__ == "__main__":
    main()
