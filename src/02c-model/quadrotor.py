from lib import *
from utils import *


class Quadrotor:

    def __init__(self, mass=1.0, inertia=None, dt=0.001, l=0.2, cf=1.0, k=0.01):
        self.mass = mass
        self.g = 9.81
        self.dt = dt
        self.l = l  # arm length
        self.cf = cf  # thrust coefficient
        self.k = k  # drag coefficient
        
        # Lumped inertia (assume diagonal)
        if inertia is None:
            inertia = np.diag([0.01, 0.01, 0.02])
        self.J = inertia
        self.J_inv = np.linalg.inv(self.J)

        # Rotor configuration (body frame)
        self.rotor = [
            {"p": np.array([ self.l, 0, 0]), "v": np.array([0,0,1]), "k":  self.k},
            {"p": np.array([ 0, self.l, 0]), "v": np.array([0,0,1]), "k": -self.k},
            {"p": np.array([-self.l, 0, 0]), "v": np.array([0,0,1]), "k":  self.k},
            {"p": np.array([ 0,-self.l, 0]), "v": np.array([0,0,1]), "k": -self.k},
        ]
        
        # Logging
        self.history = {
            'time': [], 'position': [], 'quaternion': [],
            'velocity': [], 'ang_velocity': [], 'acceleration': [], 'ang_acceleration': []
        }
        
        # State: [p_B, q_B, v_B, w_B] (13-dim: 3+4+3+3)
        self.reset()
    

    def reset(self, position=None, quaternion=None, velocity=None, ang_velocity=None):
        if position is None:
            position = np.array([0.0, 0.0, 0.0])
        if quaternion is None:
            quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # identity
        if velocity is None:
            velocity = np.array([0.0, 0.0, 0.0])
        if ang_velocity is None:
            ang_velocity = np.array([0.0, 0.0, 0.0])
        
        self.state = np.hstack([position, quaternion, velocity, ang_velocity])
        self.history = {k: [] for k in self.history.keys()}
    

    def get_position(self):
        return self.state[0:3]
    

    def get_quaternion(self):
        return self.state[3:7]
    

    def get_velocity(self):
        return self.state[7:10]
    

    def get_ang_velocity(self):
        return self.state[10:13]
    
    def get_acceleration(self):
        return self.dynamics(self.state, None)[7:10]
    
    def get_ang_acceleration(self):
        return self.dynamics(self.state, None)[10:13]
    

    def get_wrench(self, u):
        """Convert rotor inputs to force and torque"""
        f = np.zeros(3)
        m = np.zeros(3)
        for i in range(4):
            w_i = u[i]
            rotor_i = self.rotor[i]

            p_i = rotor_i['p']
            v_i = rotor_i['v']
            k_i = rotor_i['k']

            f_i = self.cf * w_i**2 * v_i
            m_i = np.cross(p_i, f_i) + k_i * w_i**2 * v_i
            
            f += f_i
            m += m_i

        return np.hstack([f, m])
    

    def set_control_input(self, force_wrench):
        """force_wrench: [fx, fy, fz, mx, my, mz] in body frame"""
        self.f = force_wrench[0:3]
        self.torque = force_wrench[3:6]
    

    def dynamics(self, state, u=None):
        """Continuous-time dynamics: dx/dt = f(x, u)"""
        p = state[0:3]
        q = state[3:7]
        v = state[7:10]
        w = state[10:13]
        
        R = rodrigues_rotation_matrix(q)
        z_W = np.array([0, 0, 1])

        # Position derivative
        p_dot = v
        
        # Quaternion derivative: q_dot = 0.5 * q ⊗ [0, w]
        w_quat = np.hstack([0, w])
        q_dot = 0.5 * quaternion_product(q, w_quat)
        
        # Linear velocity derivative: m*v_dot = -mg*z_W + R*f
        v_dot = -self.g * z_W + ( (R @ self.f) / self.mass )
        
        # Angular velocity derivative: J*w_dot = -w × J*w + m
        w_dot = self.J_inv @ (self.torque - np.cross(w, (self.J @ w)))
        
        return np.hstack([p_dot, q_dot, v_dot, w_dot])
    

    def step(self, force_wrench, u=None):
        """Simulate one step with RK4"""
        self.set_control_input(force_wrench)
        
        # RK4 integration
        self.state = rk4_step(self.dynamics, self.state, None, self.dt)
        
        # Normalize quaternion
        q = self.get_quaternion()
        self.state[3:7] = quaternion_normalize(q)
        
        # Ground reaction
        if self.get_position()[2] < 0:
            self.state[0:3] = np.array([self.state[0], self.state[1], 0])
            self.state[7:13] = 0

        return self.state
    

    def log_state(self, time):
        self.history['time'].append(time)
        self.history['position'].append(self.get_position().copy())
        self.history['quaternion'].append(self.get_quaternion().copy())
        self.history['velocity'].append(self.get_velocity().copy())
        self.history['ang_velocity'].append(self.get_ang_velocity().copy())
        self.history['acceleration'].append(self.get_acceleration().copy())
        self.history['ang_acceleration'].append(self.get_ang_acceleration().copy())


    
 
