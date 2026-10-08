"""Exploration orbit around the configured volume, facing its center."""

import numpy as np

from final_project.components.functions import _quat_to_yaw
from final_project.state_machine.state_machine import StateType


def _time_law(t, duration):
    """Quintic smoothstep and its first two time derivatives."""
    tau = np.clip(t / duration, 0.0, 1.0)
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    sd = 30.0 * tau**2 * (1.0 - tau)**2 / duration
    sdd = 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau) / duration**2
    return s, sd, sdd


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class ExploringState:
    def __init__(
        self,
        camera,
        optitrack_helper,
        follower_bodies,
        maneuver_follower,
        explore_bounds=None,
        ring_clearance=5.0,
        top_clearance=5.0,
        max_velocity=10.0,
        dt=0.05,
        explore_timeout=30.0,
        explore_velocity=None,
        explore_yaw_rate=None,
    ):
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.maneuver_follower = maneuver_follower
        self.explore_bounds = explore_bounds or ((-50.0, 50.0), (-50.0, 50.0), (0.0, 30.0))
        self.ring_clearance = ring_clearance
        self.top_clearance = top_clearance
        self.max_velocity = max_velocity
        self.dt = dt
        self.explore_timeout = explore_timeout
        self.explore_velocity = explore_velocity if explore_velocity is not None else max_velocity
        self.explore_yaw_rate = explore_yaw_rate if explore_yaw_rate is not None else 0.5

        self._planned = False
        self._path_time = 0.0
        self._connector_duration = 0.0
        self._ring_duration = 0.0
        self._ring_center = np.zeros(2)
        self._ring_radius = 0.0
        self._ring_start_angle = 0.0
        self._connector_start = np.zeros(3)
        self._ring_start = np.zeros(3)
        self._z_low = 0.0
        self._z_high = 0.0

    def _stop(self):
        self.maneuver_follower.velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    def enter(self):
        """Plan a smooth connector and one complete orbit from the current pose."""
        self._path_time = 0.0
        self._planned = False
        follower_pos, _ = self.optitrack_helper(self.follower_body_name)
        if follower_pos is not None:
            self._plan_orbit(follower_pos)

    def _plan_orbit(self, start_pos):
        (x_min, x_max), (y_min, y_max), (z_min, z_max) = self.explore_bounds
        self._ring_center = np.array([(x_min + x_max) * 0.5, (y_min + y_max) * 0.5])
        corners = np.array([
            [x_min, y_min], [x_min, y_max],
            [x_max, y_min], [x_max, y_max],
        ])
        corner_radius = float(np.max(np.linalg.norm(corners - self._ring_center, axis=1)))
        self._ring_radius = corner_radius + self.ring_clearance
        self._z_low = z_min + 1.0
        self._z_high = z_max - self.top_clearance
        if self._z_high <= self._z_low or self.explore_velocity <= 0.0:
            return

        self._connector_start = np.asarray(start_pos, dtype=float).copy()
        offset = self._connector_start[:2] - self._ring_center
        if np.linalg.norm(offset) > 1e-6:
            self._ring_start_angle = float(np.arctan2(offset[1], offset[0]))
        else:
            self._ring_start_angle = 0.0
        self._ring_start = np.array([
            self._ring_center[0] + self._ring_radius * np.cos(self._ring_start_angle),
            self._ring_center[1] + self._ring_radius * np.sin(self._ring_start_angle),
            self._z_low,
        ])

        connector_distance = float(np.linalg.norm(self._ring_start - self._connector_start))
        self._connector_duration = max(
            self.dt,
            1.875 * connector_distance / self.max_velocity,
        )
        self._ring_duration = 2.0 * np.pi * self._ring_radius / self.explore_velocity
        self._planned = True

    def _connector_state(self, elapsed):
        s, sd, sdd = _time_law(elapsed, self._connector_duration)
        delta = self._ring_start - self._connector_start
        return self._connector_start + delta * s, delta * sd, delta * sdd

    def _orbit_state(self, elapsed):
        progress = np.clip(elapsed / self._ring_duration, 0.0, 1.0)
        theta_rate = 2.0 * np.pi / self._ring_duration
        theta = self._ring_start_angle + 2.0 * np.pi * progress

        if progress <= 0.5:
            altitude_time = progress * self._ring_duration
            s, sd, sdd = _time_law(altitude_time, self._ring_duration * 0.5)
            z = self._z_low + (self._z_high - self._z_low) * s
            vz = (self._z_high - self._z_low) * sd
            az = (self._z_high - self._z_low) * sdd
        else:
            altitude_time = (progress - 0.5) * self._ring_duration
            s, sd, sdd = _time_law(altitude_time, self._ring_duration * 0.5)
            z = self._z_high - (self._z_high - self._z_low) * s
            vz = -(self._z_high - self._z_low) * sd
            az = -(self._z_high - self._z_low) * sdd

        cos_theta, sin_theta = np.cos(theta), np.sin(theta)
        radius = self._ring_radius
        position = np.array([
            self._ring_center[0] + radius * cos_theta,
            self._ring_center[1] + radius * sin_theta,
            z,
        ])
        velocity = np.array([
            -radius * sin_theta * theta_rate,
            radius * cos_theta * theta_rate,
            vz,
        ])
        acceleration = np.array([
            -radius * cos_theta * theta_rate**2,
            -radius * sin_theta * theta_rate**2,
            az,
        ])
        return position, velocity, acceleration

    def update(self):
        leader_pos, _ = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            self._stop()
            return None

        pos_in_body = self.camera.measure(leader_pos, follower_pos, follower_quat)
        if self.camera.contains(pos_in_body):
            self._stop()
            return StateType.TRACKING

        if not self._planned:
            self._stop()
            return StateType.IDLE

        total_duration = self._connector_duration + self._ring_duration
        if self._path_time >= total_duration:
            self._stop()
            return StateType.IDLE

        if self._path_time < self._connector_duration:
            _, velocity, acceleration = self._connector_state(self._path_time)
        else:
            _, velocity, acceleration = self._orbit_state(self._path_time - self._connector_duration)

        follower_yaw = _quat_to_yaw(follower_quat)
        to_center = self._ring_center - np.asarray(follower_pos[:2], dtype=float)
        if np.linalg.norm(to_center) > 1e-6:
            camera_yaw = float(np.arctan2(to_center[1], to_center[0]))
            desired_yaw = _wrap(camera_yaw - self.camera.yaw_offset)
            yaw_rate = np.clip(
                1.5 * _wrap(desired_yaw - follower_yaw),
                -self.explore_yaw_rate,
                self.explore_yaw_rate,
            )
        else:
            yaw_rate = 0.0

        self.maneuver_follower.velocity(
            velocity[0], velocity[1], velocity[2], yaw_rate,
            acceleration[0], acceleration[1], acceleration[2], 0,
        )
        self._path_time += self.dt

        if self._path_time >= total_duration:
            self._stop()
            return StateType.IDLE

        return None
