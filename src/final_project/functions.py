import genomix
import os
import random
import math
import time


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



def setup_one(rotorcraft, pom, nhfc, maneuver,
              rc_name, pom_name, nhfc_name, maneuver_name,
              serial, mocap_body):

  #############################################
  #                  MANEUVER                 #
  #############################################

  # maneuver reads the current robot state from pom, same source as nhfc
  maneuver.connect_port({
    'local': 'state', 'remote': pom_name + '/frame/robot'
  })

  # configure the free-space bounds within which trajectories can be planned
  maneuver.set_bounds({
      'xmin': -10, 'xmax': 10,
      'ymin': -10, 'ymax': 10,
      'zmin': 0,   'zmax': 10,
      'yawmin': -3.14, 'yawmax': 3.14
  })


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
  nhfc.set_servo_gain({ 'gain': {
    'Kpxy': 5, 'Kpz': 5, 'Kqxy': 4, 'Kqz': 0.1,
    'Kvxy': 6, 'Kvz': 6, 'Kwxy': 1, 'Kwz': 0.1,
    'Kixy': 0, 'Kiz': 0
  }})

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
  setup_one(rotorcraft_l, pom_l, nhfc_l, maneuver_l,
            'rotorcraft_1', 'pom_1', 'nhfc_1', 'maneuver_1',
            '/tmp/pty-qr4_leading', 'QR4_leading')

  # follower  (adjust serial device and mocap body name to your setup)
  setup_one(rotorcraft_f, pom_f, nhfc_f, maneuver_f,
            'rotorcraft_2', 'pom_2', 'nhfc_2', 'maneuver_2',
            '/tmp/pty-qr4_following', 'QR4_following')


# --- start ----------------------------------------------------------------
#
# Spin the motors and servo on current position for one drone. The 'tag'
# keeps the per-drone log files separate.
def start_one(rotorcraft, pom, nhfc, maneuver, tag):
  pom.log_state('/tmp/pom_%s.log' % tag)
  pom.log_measurements('/tmp/pom-measurements_%s.log' % tag)

  rotorcraft.log('/tmp/rotorcraft_%s.log' % tag)
  rotorcraft.start()
  rotorcraft.servo(ack=True) # this runs until stopped or input error

  nhfc.log('/tmp/nhfc_%s.log' % tag)
  #nhfc.set_current_position() # hover on current position
  nhfc.servo(ack=True)       # start nhfc's own control loop (reads state+reference, drives rotor_input)

  maneuver.log('/tmp/maneuver_%s.log' % tag)
  maneuver.set_current_state() # this runs until stopped or input error


# Spin the motors and servo on current position. To be called interactively
def start():
  optitrack.set_logfile('/tmp/opti.log')

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


def not_random_trajectory(n_points=10):
  maneuver_f.waypoint({
    'x': 1, 'y': 1, 'z': 3, 'yaw': 0, 'duration': 10,
    'vx': 1, 'vy': 1, 'vz': 1, 'wz': 0, 'ax': 1, 'ay': 1, 'az': 1
  })
  maneuver_f.waypoint({
    'x': 0, 'y': 0, 'z': 0, 'yaw': 0, 'duration': 10,
    'vx': 0, 'vy': 0, 'vz': 0, 'wz': 0, 'ax': 0, 'ay': 0, 'az': 0
  })
  
  maneuver_f.wait()


def random_trajectory(n_points):
  for i in range(n_points):
    maneuver_l.waypoint({
      'x': random.uniform(-5, 5), 
      'y': random.uniform(-5, 5),
      'z': random.uniform(0, 5), 
      'yaw': 0,
      'duration': 15,
      'vx': 1, 
      'vy': 1,
      'vz': 1, 
      'wz': 0,
      'ax': 1,
      'ay': 1,
      'az': 1
    })

    maneuver_f.waypoint({
        'x': random.uniform(-5, 5), 
        'y': random.uniform(-5, 5),
        'z': random.uniform(0, 5), 
        'yaw': 0,
        'duration': 15,
        'vx': 1, 
        'vy': 1,
        'vz': 1, 
        'wz': 0,
        'ax': 1,
        'ay': 1,
        'az': 1
      })

  maneuver_f.waypoint({
      'x': 0, 'y': 0, 'z': 0, 'yaw': 0, 'duration': 10,
      'vx': 0, 'vy': 0, 'vz': 0, 'wz': 0, 'ax': 0, 'ay': 0, 'az': 0
    })

  maneuver_l.waypoint({
      'x': 2, 'y': 2, 'z': 0, 'yaw': 0, 'duration': 10,
      'vx': 0, 'vy': 0, 'vz': 0, 'wz': 0, 'ax': 0, 'ay': 0, 'az': 0
    })
  
  maneuver_l.wait()
  maneuver_f.wait()


## interactively, one can start the simulation with
# setup()
# start()
## and then for instance set a desired position with
# nhfc.set_position(0, 0, 1, 0)
## to stop, use
# stop
