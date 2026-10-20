import genomix
from genomix.event import GenoMError
import os
import random
import numpy as np
from final_project.config import CONFIG, genom_log_path

NHFC_GAINS = CONFIG['nhfc_gains']


# this connects to components running on the same host (localhost)
g = genomix.connect()
# to instead control components running on the remote computer "hostname" use
# g = genomix.connect('hostname')

# adapt path to your setup
g.rpath(os.environ['HOME'] + '/openrobots/lib/genom/pocolibs/plugins')

# load components clients
optitrack = g.load('optitrack')

rotorcraft_l = g.load('rotorcraft', '-i', 'rotorcraft_1')
rotorcraft_f = g.load('rotorcraft', '-i', 'rotorcraft_2')
pom_l = g.load('pom', '-i', 'pom_1')
pom_f = g.load('pom', '-i', 'pom_2')
nhfc_l = g.load('nhfc', '-i', 'nhfc_1')
nhfc_f = g.load('nhfc', '-i', 'nhfc_2')
maneuver_l = g.load('maneuver', '-i', 'maneuver_1')
maneuver_f = g.load('maneuver', '-i', 'maneuver_2')



# Configure one drone (same setup for leader and follower).
def setup_drone(rotorcraft, pom, nhfc, maneuver,
                rc_name, pom_name, nhfc_name, maneuver_name,
                serial, mocap_body):
  # maneuver reads the current robot state from pom, same source as nhfc
  maneuver.connect_port({
    'local': 'state', 'remote': pom_name + '/frame/robot'
  })

  # configure the free-space bounds within which trajectories can be planned
  maneuver.set_bounds(dict(CONFIG['maneuver_bounds']))


  #############################################
  #                ROTORCRAFT                 #
  #############################################

  # connect to the simulated quadrotor
  rotorcraft.connect({'serial': serial, 'baud': 0})

  # get IMU at 1kHz and motor data at 20Hz
  rotorcraft.set_sensor_rate({'rate': {
    'imu': 1000, 'mag': 0, 'motor': 20, 'battery': 1
  }})

  # Filter IMU: 20Hz cut-off frequency for gyroscopes and 5Hz for
  # accelerometers. This is important for cancelling vibrations.
  rotorcraft.set_imu_filter({
    'gfc': [20, 20, 20], 'afc': [5, 5, 5], 'mfc': [20, 20, 20]
  })

  # read propellers velocities from nhfc controller
  rotorcraft.connect_port({
    'local': 'rotor_input', 'remote': nhfc_name + '/rotor_input'
  })


  #############################################
  #                   NHFC                    #
  #############################################
  #
  # configure quadrotor geometry: 4 rotors, not tilted, 23cm arms, needed to compute the allocation matrix
  nhfc.set_gtmrp_geom({
    'rotors': 4, 'cx': 0, 'cy': 0, 'cz': 0, 'armlen': 0.23, 'mass': 1.28,
    'rx':0, 'ry': 0, 'rz': -1, 'cf': 6.5e-4, 'ct': 1e-5
  })

  # emergency descent parameters
  nhfc.set_emerg({'emerg': {
    'descent': 0.1, 'dx': 0.5, 'dq': 1, 'dv': 3, 'dw': 3
  }})

  # read planned trajectory reference from maneuver
  nhfc.connect_port({
    'local': 'reference', 'remote': maneuver_name + '/desired'
  })

  # PID tuning
  nhfc.set_saturation({'sat': {'x': 1, 'v': 1, 'ix': 0}})
  nhfc.set_servo_gain({'gain': dict(NHFC_GAINS)})  # copy: genomix empties the dict it receives

  # use tilt-prioritized controller
  nhfc.set_control_mode({'att_mode': '::nhfc::tilt_prioritized'})

  # read measured propeller velocities from rotorcraft
  nhfc.connect_port({
    'local': 'rotor_measure', 'remote': rc_name + '/rotor_measure'
  })

  # read current state from pom
  nhfc.connect_port({
    'local': 'state', 'remote': pom_name + '/frame/robot'
  })


  #############################################
  #                    POM                    #
  #############################################
  #
  # configure kalman filter
  pom.set_prediction_model('::pom::constant_acceleration')
  pom.set_process_noise({'max_jerk': 100, 'max_dw': 50})

  # allow sensor data up to 250ms old
  pom.set_history_length({'history_length': 0.25})

  # configure magnetic field
  pom.set_mag_field({'magdir': {
    'x': 23.8e-06, 'y': -0.4e-06, 'z': -39.8e-06
  }})

  # read IMU and magnetometers from rotorcraft
  pom.connect_port({'local': 'measure/imu', 'remote': rc_name + '/imu'})
  pom.add_measurement('imu')
  pom.connect_port({'local': 'measure/mag', 'remote': rc_name + '/mag'})
  pom.add_measurement('mag')

  # read position and orientation from optitrack
  pom.connect_port({
    'local': 'measure/mocap', 'remote': 'optitrack/bodies/' + mocap_body
  })
  pom.add_measurement('mocap')

# configure both quadrotors, to be called interactively
def setup():

  #############################################
  #                OPTITRACK                  #
  #############################################
  #
  # optitrack is a single shared system: connect it once, it streams all bodies
  optitrack.connect({
    'host': 'localhost', 'host_port': '1509', 'mcast': '', 'mcast_port': '0'
  })

  # leader
  setup_drone(rotorcraft_l, pom_l, nhfc_l, maneuver_l,
              'rotorcraft_1', 'pom_1', 'nhfc_1', 'maneuver_1',
              '/tmp/pty-qr4_leading', 'QR4_leading')

  # follower  (adjust serial device and mocap body name to your setup)
  setup_drone(rotorcraft_f, pom_f, nhfc_f, maneuver_f,
              'rotorcraft_2', 'pom_2', 'nhfc_2', 'maneuver_2',
              '/tmp/pty-qr4_following', 'QR4_following')


# --- start ----------------------------------------------------------------
#
# Spin the motors and servo on current position for one drone. The 'tag'
# keeps the per-drone log files separate.
def start_one(rotorcraft, pom, nhfc, maneuver, tag):
  pom.log_state(genom_log_path('pom_%s.log' % tag))
  pom.log_measurements(genom_log_path('pom-measurements_%s.log' % tag))

  rotorcraft.log(genom_log_path('rotorcraft_%s.log' % tag))
  rotorcraft.start()
  rotorcraft.servo(ack=True) # this runs until stopped or input error

  nhfc.log(genom_log_path('nhfc_%s.log' % tag))
  nhfc.servo(ack=True)       # start nhfc's own control loop (reads state+reference, drives rotor_input)

  maneuver.log(genom_log_path('maneuver_%s.log' % tag))
  maneuver.set_current_state() # initialize the planner from the current pom state (returns immediately)


# Spin the motors and servo on current position. To be called interactively
def start():
  optitrack.set_logfile(genom_log_path('opti.log'))

  start_one(rotorcraft_l, pom_l, nhfc_l, maneuver_l, 'leader')
  start_one(rotorcraft_f, pom_f, nhfc_f, maneuver_f, 'follower')

# --- stop -----------------------------------------------------------------
#
# Stop motors for one drone.
def stop_one(rotorcraft, pom, nhfc, maneuver):
  rotorcraft.stop()
  rotorcraft.log_stop()

  nhfc.stop()
  nhfc.log_stop()

  maneuver.log_stop()

  pom.log_stop()


# Stop motors. To be called interactively
def stop():
  stop_one(rotorcraft_l, pom_l, nhfc_l, maneuver_l)
  stop_one(rotorcraft_f, pom_f, nhfc_f, maneuver_f)

  optitrack.unset_logfile()



def random_trajectory(n_points, start, speed, z_range=(2.0, 6.0), xy_range=10.0,
                      maneuver=None, min_segment=2.0, max_retries=8):
  """
  Queues in maneuver (default maneuver_l) n_points random waypoints plus a final
  point at rest, all with altitude in z_range (the leader never goes below z_min).
  - duration of each segment = distance / speed (at least min_segment s):
    average velocity ≈ speed, regardless of how far apart the points are
  - velocity when passing a waypoint = speed along the Catmull-Rom tangent
    (previous → next direction), so the motion is smooth; 0 on the final point
  If maneuver rejects a segment because of the limits (e_limits), its duration is extended.
  Returns the total duration [s].
  """
  maneuver = maneuver_l if maneuver is None else maneuver
  z_min, z_max = z_range
  pts = [np.asarray(start, dtype=float)]
  for _ in range(n_points):
    pts.append(np.array([random.uniform(-xy_range, xy_range),
                         random.uniform(-xy_range, xy_range),
                         random.uniform(z_min, z_max)]))
  pts.append(np.array([random.uniform(-xy_range, xy_range),
                       random.uniform(-xy_range, xy_range), z_min]))

  total = 0.0
  for i in range(1, len(pts)):
    duration = max(min_segment, np.linalg.norm(pts[i] - pts[i - 1]) / speed)
    if i < len(pts) - 1:
      tangent = pts[i + 1] - pts[i - 1]
      v = speed * tangent / max(np.linalg.norm(tangent), 1e-6)
    else:
      v = np.zeros(3)                       # final point: at rest
    # maneuver rejects (e_limits) a segment that, with this duration, would violate the
    # leader velocity/acceleration limits: retry with a longer one, at worst duration=0
    # (as fast as possible within the limits)
    for _ in range(max_retries):
      try:
        maneuver.waypoint({
          'x': pts[i][0], 'y': pts[i][1], 'z': pts[i][2], 'yaw': 0,
          'duration': 0,
          'vx': 0, 'vy': 0, 'vz': 0, 'wz': 0,
          'ax': 0, 'ay': 0, 'az': 0
        })
        break
      except GenoMError as e:
        if 'e_limits' not in str(e):
          raise
        duration *= 1.3
    else:
      maneuver.waypoint({
        'x': pts[i][0], 'y': pts[i][1], 'z': pts[i][2], 'yaw': 0,
        'duration': 0,
        'vx': v[0], 'vy': v[1], 'vz': v[2], 'wz': 0,
        'ax': 0, 'ay': 0, 'az': 0
      })
      duration = np.linalg.norm(pts[i] - pts[i - 1]) / speed   # estimate for max_loops
    total += duration
  return total


def _quat_to_yaw(quat):
  """Converts a quaternion (qw, qx, qy, qz) to yaw."""
  qw, qx, qy, qz = quat
  yaw = np.arctan2(
    2 * (qw * qz + qx * qy),
    1 - 2 * (qy**2 + qz**2)
  )
  return yaw
