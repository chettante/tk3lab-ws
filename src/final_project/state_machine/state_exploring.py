"""
ExploringState: il follower orbita attorno al volume di esplorazione guardandone il centro.

Percorso: un tratto di collegamento rettilineo fino all'anello, poi un giro completo
dell'anello salendo da z_low a z_high nella prima metà e scendendo nella seconda.

Transizioni:
  - → TRACKING se il leader entra nel FOV
  - → IDLE a fine giro (leader non trovato) o se il percorso non è pianificabile
"""

import logging

import numpy as np

from final_project.components.functions import _quat_to_yaw
from final_project.components.Research import time_law, _ptp, _ptp_duration, _wrap
from final_project.state_machine.state_machine import StateType

logger = logging.getLogger("ExploringState")


class ExploringState:
    def __init__(
        self,
        camera,
        optitrack_helper,
        follower_bodies,
        maneuver_follower,
        explore_bounds,
        ring_clearance,
        top_clearance,
        max_velocity,
        max_acc,
        dt,
        explore_velocity,
        explore_yaw_rate,
    ):
        """
        Args:
            explore_bounds: ((x_min, x_max), (y_min, y_max), (z_min, z_max)) del volume
            ring_clearance: m, margine dell'anello oltre gli spigoli del volume
            top_clearance: m, distanza della quota massima dal tetto del volume
            max_velocity, max_acc: limiti del tratto di collegamento
            explore_velocity: m/s, velocità lungo l'anello
            explore_yaw_rate: rad/s, yaw rate massimo per puntare il centro
        """
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.maneuver_follower = maneuver_follower
        self.explore_bounds = explore_bounds
        self.ring_clearance = ring_clearance
        self.top_clearance = top_clearance
        self.max_velocity = max_velocity
        self.max_acc = max_acc
        self.dt = dt
        self.explore_velocity = explore_velocity
        self.explore_yaw_rate = explore_yaw_rate

        self._planned = False           # piano valido per questa esplorazione
        self._path_time = 0.0           # tempo trascorso lungo il percorso

    def _stop(self):
        self.maneuver_follower.velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    def enter(self):
        """Chiamato dalla StateMachine a ogni ingresso in EXPLORING: ripianifica dalla posa attuale."""
        self._path_time = 0.0
        self._planned = False
        follower_pos, _ = self.optitrack_helper(self.follower_body_name)
        if follower_pos is not None:
            self._plan_orbit(follower_pos)
        if self._planned:
            logger.info(
                f"Exploration planned ({self._total_duration:.1f}s): "
                f"ring r={self._ring_radius:.1f}m, z {self._z_low:.1f}-{self._z_high:.1f}m"
            )
        else:
            logger.warning("Exploration could not be planned (no follower pose or invalid bounds)")

    def _plan_orbit(self, start_pos):
        """Anello che contiene il volume + tratto di collegamento dalla posizione attuale."""
        (x_min, x_max), (y_min, y_max), (z_min, z_max) = self.explore_bounds
        self._z_low = z_min + 1.0
        self._z_high = z_max - self.top_clearance
        if self._z_high <= self._z_low or self.explore_velocity <= 0.0:
            return

        # anello centrato nel volume, raggio = semidiagonale + margine
        self._ring_center = np.array([(x_min + x_max) * 0.5, (y_min + y_max) * 0.5])
        self._ring_radius = 0.5 * np.hypot(x_max - x_min, y_max - y_min) + self.ring_clearance

        # si entra nell'anello nel punto più vicino al follower, alla quota z_low
        self._connector_start = np.asarray(start_pos, dtype=float).copy()
        offset = self._connector_start[:2] - self._ring_center
        self._ring_start_angle = float(np.arctan2(offset[1], offset[0])) if np.linalg.norm(offset) > 1e-6 else 0.0
        self._ring_start = np.array([
            *(self._ring_center + self._ring_radius * np.array([np.cos(self._ring_start_angle),
                                                                np.sin(self._ring_start_angle)])),
            self._z_low,
        ])

        self._connector_duration = _ptp_duration(self._connector_start, self._ring_start,
                                                 self.max_velocity, self.max_acc)
        self._ring_duration = 2.0 * np.pi * self._ring_radius / self.explore_velocity
        self._total_duration = self._connector_duration + self._ring_duration
        self._planned = True

    def _orbit_state(self, elapsed):
        """Velocità e accelerazione sull'anello al tempo `elapsed` dall'ingresso nell'anello."""
        theta_rate = 2.0 * np.pi / self._ring_duration
        theta = self._ring_start_angle + theta_rate * elapsed

        # quota: smoothstep z_low → z_high nella prima metà del giro, ritorno nella seconda
        half = 0.5 * self._ring_duration
        rising = elapsed <= half
        _, sd, sdd = time_law(elapsed if rising else elapsed - half, half)
        dz = (self._z_high - self._z_low) * (1.0 if rising else -1.0)

        c, s = np.cos(theta), np.sin(theta)
        r = self._ring_radius
        velocity = np.array([-r * s * theta_rate, r * c * theta_rate, dz * sd])
        acceleration = np.array([-r * c * theta_rate**2, -r * s * theta_rate**2, dz * sdd])
        return velocity, acceleration

    def _yaw_rate_to_center(self, follower_pos, follower_quat):
        """Yaw rate (P saturato) che punta l'asse ottico della camera sul centro dell'anello."""
        to_center = self._ring_center - np.asarray(follower_pos[:2], dtype=float)
        if np.linalg.norm(to_center) < 1e-6:
            return 0.0
        desired_yaw = _wrap(np.arctan2(to_center[1], to_center[0]) - self.camera.yaw_offset)
        yaw_err = _wrap(desired_yaw - _quat_to_yaw(follower_quat))
        return float(np.clip(1.5 * yaw_err, -self.explore_yaw_rate, self.explore_yaw_rate))

    def update(self):
        """
        1. Se il leader è nel FOV → TRACKING (senza stop: il tracking parte dal moto attuale)
        2. A fine percorso (o senza piano) ferma il drone → IDLE
        3. Altrimenti manda velocità/accelerazione del collegamento o dell'anello + yaw verso il centro
        """
        leader_pos, _ = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            self._stop()
            return None

        pos_in_body = self.camera.measure(leader_pos, follower_pos, follower_quat)
        if self.camera.contains(pos_in_body):
            logger.info("Leader detected in FOV! Transitioning to TRACKING")
            return StateType.TRACKING

        if not self._planned:
            logger.warning("Exploration not planned, transitioning to IDLE")
            self._stop()
            return StateType.IDLE

        if self._path_time >= self._total_duration:
            logger.warning("Exploration finished without finding the leader, transitioning to IDLE")
            self._stop()
            return StateType.IDLE

        if self._path_time < self._connector_duration:
            _, velocity, acceleration = _ptp(self._path_time, self._connector_duration,
                                             self._connector_start, self._ring_start)
        else:
            velocity, acceleration = self._orbit_state(self._path_time - self._connector_duration)

        self.maneuver_follower.velocity(
            velocity[0], velocity[1], velocity[2],
            self._yaw_rate_to_center(follower_pos, follower_quat),
            acceleration[0], acceleration[1], acceleration[2], 0,
        )
        self._path_time += self.dt
        return None
