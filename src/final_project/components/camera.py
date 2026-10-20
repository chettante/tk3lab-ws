import numpy as np
import time
import logging
from genomix.event import GenoMError
from final_project.components.velocity_estimator import VelocityEstimator


logger = logging.getLogger("FOVCamera")


def quat_to_rot(q):
    """Body→world rotation matrix from a quaternion (qw, qx, qy, qz)."""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


class FOVPyramid:
    """
    FOV of a pinhole camera (like the Gazebo one): a truncated pyramid with a
    rectangular base and its apex in the camera. Optical axis = drone x axis rotated
    by yaw_offset around z (camera mounting).
      - horizontal half-angle h and vertical half-angle v (optical axis ↔ pyramid faces)
      - range = depth along the optical axis, between the near and far (max_range) planes
    gimbal=True: stabilized camera, the measurement rotates only with the follower yaw;
    gimbal=False: camera rigidly fixed to the body (as in Gazebo), rotates with roll/pitch/yaw.
    """

    def __init__(self, half_angle_h_deg: float = 45.0, half_angle_v_deg: float = 36.87,
                 max_range: float = 10.0, gimbal: bool = False, near: float = 0.1,
                 yaw_offset_deg: float = 0.0):
        """
        Args:
            half_angle_h_deg: horizontal half-angle (degrees, optical axis ↔ side face)
            half_angle_v_deg: vertical half-angle (degrees, optical axis ↔ top/bottom face)
            max_range: far plane, maximum depth (meters)
            gimbal: True = yaw only, False = full attitude
            near: near plane (meters)
            yaw_offset_deg: camera mounting yaw relative to the drone x axis
        """
        self.max_range = max_range
        self.near = near
        self.gimbal = gimbal
        self.yaw_offset = np.radians(yaw_offset_deg)    # rad, camera heading = drone yaw + offset
        self.tan_h = np.tan(np.radians(half_angle_h_deg))
        self.tan_v = np.tan(np.radians(half_angle_v_deg))

    def contains(self, point_in_body_frame: np.ndarray) -> bool:
        """
        True if the point is inside the FOV.

        Args:
            point_in_body_frame: [x forward, y left, z up] in the camera frame
        """
        x, y, z = np.asarray(point_in_body_frame, dtype=float)
        if not (self.near <= x <= self.max_range):
            return False
        return abs(y) <= x * self.tan_h and abs(z) <= x * self.tan_v

    def measure(self, leader_pos, follower_pos, follower_quat) -> np.ndarray:
        """
        "Camera measurement": leader position in the camera frame.
        With gimbal it rotates only with the follower yaw, without it with the full attitude.
        """
        rel = np.asarray(leader_pos, dtype=float) - np.asarray(follower_pos, dtype=float)
        R = quat_to_rot(follower_quat)
        if self.gimbal:
            # yaw only: drone yaw + camera mounting yaw
            return self.global_to_body(rel, np.zeros(3), np.arctan2(R[1, 0], R[0, 0]) + self.yaw_offset)
        body = R.T @ rel                                # leader in the drone axes
        # from the drone axes to the camera axes (rotated by yaw_offset around z)
        return self.global_to_body(body, np.zeros(3), self.yaw_offset)

    def to_world(self, point_in_cam, follower_pos, follower_quat) -> np.ndarray:
        """Inverse of measure(): leader position in world from the camera measurement."""
        R = quat_to_rot(follower_quat)
        if self.gimbal:
            yaw = np.arctan2(R[1, 0], R[0, 0]) + self.yaw_offset
            rel = self.global_to_body(point_in_cam, np.zeros(3), -yaw)
        else:
            rel = R @ self.global_to_body(point_in_cam, np.zeros(3), -self.yaw_offset)
        return np.asarray(follower_pos, dtype=float) + rel

    def get_yaw_error(self, pos_relative, follower_yaw=0.0):
        """
        Computes the heading error: how much to rotate to center the leader

        Args:
            pos_relative: [dx, dy, dz] leader position in the global frame
                        (not the body frame!)
            follower_yaw: current follower orientation (radians)

        Returns:
            yaw_error: rotation angle (radians)
                    positive = rotate CCW (left)
                    negative = rotate CW (right)
        """
        # Angle of the leader relative to the follower
        leader_angle = np.arctan2(pos_relative[1], pos_relative[0])

        # Error = where it should look - where it looks
        yaw_error = leader_angle - follower_yaw

        # Wrap to [-π, π]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

        return yaw_error

    def global_to_body(self, pos_leader: np.ndarray, pos_follower: np.ndarray, follower_yaw: float) -> np.ndarray:
        """Rotates (pos_leader - pos_follower) by -follower_yaw around z
        (global frame → follower yaw frame; also used as a generic z rotation)."""
        pos_rel = pos_leader - pos_follower
        cos_y, sin_y = np.cos(follower_yaw), np.sin(follower_yaw)

        R_w2b = np.array([
            [ cos_y,  sin_y, 0.0],
            [-sin_y,  cos_y, 0.0],
            [   0.0,    0.0, 1.0]
        ])
        return R_w2b @ pos_rel


class TrackingController:
    """
    The follower keeps the leader centered in the FOV: yaw pointed at the leader (bearing),
    position follow_distance behind the leader along the bearing, at the same altitude.
    """

    def __init__(self, follower, velocity_estimator=None,
                 kp_xy=0.3, kp_z=0.6, follow_distance=1.5, cmd_acc_max=1.0,
                 yaw_priority_start_deg=25.0, yaw_priority_end_deg=45.0,
                 vprio_start_deg=18.0, vprio_end_deg=29.0,
                 max_velocity=2.5, *, yaw_rate_max):
        self.follower = follower
        # yaw priority: above start the xy correction decreases, at end it is zero
        self.yaw_prio = np.radians([yaw_priority_start_deg, yaw_priority_end_deg])
        # vertical priority: above start the in-plane approach decreases, at end it is zero
        self.v_prio = np.radians([vprio_start_deg, vprio_end_deg])
        self.follow_distance = follow_distance   # m, horizontal distance from the leader
        self.cmd_acc_max = cmd_acc_max           # m/s², max change of the velocity command
        self.max_velocity = max_velocity         # m/s, drone limit (same as the leader)
        self.velocity_estimator = velocity_estimator or VelocityEstimator()
        self.kp = np.array([kp_xy, kp_xy, kp_z])
        self.yaw_rate_max = yaw_rate_max
        self.t_prev = None
        self.last_cmd = np.zeros(3)

    def reset(self, follower_vel=None):
        """To be called when entering TRACKING. With follower_vel (measured velocity) the
        rate limiter starts from the actual motion instead of from rest (bumpless transfer)."""
        self.t_prev = None
        self.last_cmd[:] = 0 if follower_vel is None else np.asarray(follower_vel, dtype=float)

    def compute_command(self, leader_pos, follower_pos, follower_yaw,
                        camera, leader_vel, kp_angle=1.0, follower_vel=None):
        now = time.monotonic()
        dt = 0.05 if self.t_prev is None else np.clip(now - self.t_prev, 1e-3, 0.2)
        self.t_prev = now

        leader_pos = np.asarray(leader_pos, dtype=float)
        leader_vel = np.zeros(3) if leader_vel is None else np.asarray(leader_vel)

        # desired point: follow_distance behind the leader along the horizontal
        # follower→leader bearing (same altitude as the leader)
        rel = leader_pos - follower_pos
        d_xy = np.linalg.norm(rel[:2])
        if d_xy > 1e-3:
            u = rel[:2] / d_xy
        else:                                   # overlapping: back off along the yaw
            u = np.array([np.cos(follower_yaw), np.sin(follower_yaw)])
        target = leader_pos - self.follow_distance * np.array([u[0], u[1], 0.0])

        # P + feedforward: v = v_leader + kp·(target − p)
        v_corr = self.kp * (target - follower_pos)

        # vertical priority: the camera is rigid, the leader stays vertically in frame only by
        # changing altitude or distance (angle ≈ atan(Δz/d)). At close range, approaching makes
        # the angle grow a lot: near the vertical edge the approach component is cancelled
        # (backing off, if d < follow_distance, stays full), the altitude correction stays full
        elev = abs(np.arctan2(rel[2], d_xy))
        lo_v, hi_v = self.v_prio
        v_prio = np.clip((hi_v - elev) / (hi_v - lo_v), 0.0, 1.0)
        approach = v_corr[:2] @ u
        if approach > 0.0:
            v_corr[:2] -= (1.0 - v_prio) * approach * u

        # yaw priority: if the leader gets close to the FOV edge, first rotate
        # to re-center it and only then correct the in-plane position
        yaw_err = camera.get_yaw_error(rel, follower_yaw)
        lo, hi = self.yaw_prio
        prio = np.clip((hi - abs(yaw_err)) / (hi - lo), 0.0, 1.0)
        v_corr[:2] *= prio

        v_cmd = leader_vel + v_corr             # full FF (gain = 1)

        # rate limiter: the command cannot change by more than cmd_acc_max*dt per tick
        dv = v_cmd - self.last_cmd
        dv_max = self.cmd_acc_max * dt
        n_dv = np.linalg.norm(dv)
        if n_dv > dv_max:
            v_cmd = self.last_cmd + dv * (dv_max / n_dv)

        # drone velocity limit (same as the leader)
        n = np.linalg.norm(v_cmd)
        if n > self.max_velocity:
            v_cmd = v_cmd * (self.max_velocity / n)

        # yaw: leader centered in the FOV = P on the bearing + feedforward of its angular velocity
        # relative velocity computed with the MEASURED follower velocity: the last command
        # can differ a lot from the actual motion and give a FF in the wrong direction
        v_f = self.last_cmd if follower_vel is None else np.asarray(follower_vel)
        rel_v = leader_vel - v_f
        r2 = max(rel[0]**2 + rel[1]**2, 0.25)
        bearing_rate = (rel[0]*rel_v[1] - rel[1]*rel_v[0]) / r2
        yaw_cmd = np.clip(kp_angle * yaw_err + bearing_rate,
                          -self.yaw_rate_max, self.yaw_rate_max)

        self.last_cmd = v_cmd.copy()
        return v_cmd, yaw_cmd

    def send_command(self, velocity_cmd, yaw_cmd: float = 0.0, max_acc: float = 3.0):
        """Sends the command to the follower (already saturated in compute_command)."""
        velocity_cmd = np.asarray(velocity_cmd, dtype=float)
        try:
            self.follower.velocity(
                vx=float(velocity_cmd[0]),
                vy=float(velocity_cmd[1]),
                vz=float(velocity_cmd[2]),
                wz=float(yaw_cmd),
                ax=max_acc, ay=max_acc, az=max_acc,
                duration=0.0,
            )
        except GenoMError as e:
            # e.g. ::genom::interrupted: a single lost command must not stop the loop
            logger.warning(f"follower.velocity failed: {e}")
