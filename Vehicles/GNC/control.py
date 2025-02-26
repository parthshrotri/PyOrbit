import quaternion
import numpy as np

class Controller():
    def __init__(self, orient_gains):
        self.orient_kp      = orient_gains[0]
        self.orient_kd     = orient_gains[1]

    def get_torque_cmd(self, rot_state, q_target):
        q_now   = quaternion.as_quat_array(rot_state[0:4])
        omega   = rot_state[4:7]

        # Get the control gains
        k_p = self.orient_kp
        k_d = self.orient_kd

        # Compute the quaternion error
        dq  =   q_now*q_target.inverse()

        # Compute the control input
        u   =   -k_p*np.array([dq.x, dq.y, dq.z]) - k_d*omega
        return u
    
    def get_thrust_command(self, state):
        return np.array([0, 0, 0])
