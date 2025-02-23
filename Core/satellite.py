import numpy as np
import quaternion

import utils.attitude_ref as att_ref

class Satellite():
    def __init__(self, name, state, mass_prop, visual_prop):
        # Populate the properties
        self.mass                   = mass_prop[0]
        self.J                      = mass_prop[1]
        self.name                   = name

        # Set the visualization properties
        self.model                  = visual_prop[0]
        self.axis_order             = visual_prop[1]
        self.lights                 = visual_prop[2]

        # Set the control gains for attitude control
        self.orientation_control_gains  = np.array([7.0, 35])

        # Initialize the history arrays
        self.init_state             = state
        self.state_history          = np.array([self.init_state])
        self.target_orient_history  = np.array([state[6:10]])

    def get_state(self):
        return self.state_history[-1,:]
    
    def get_linear_state(self):
        return self.state_history[-1,0:6]
    
    def get_pos(self):
        return self.state_history[-1,0:3]
    
    def get_vel(self):
        return self.state_history[-1,3:6]
    
    def get_rotational_state(self):
        return self.state_history[-1,6:13]
    
    def get_quat(self):
        return quaternion.as_quat_array(self.state_history[-1,6:10])
    
    def get_ang_vel(self):
        return self.state_history[-1,10:13]
    
    def update_state_history(self, state):
        self.state_history  = np.vstack((self.state_history, state))
    
    def update_target_state_hist(self, target_orient):
        self.target_orient_history  = np.vstack((self.target_orient_history, 
                                                 quaternion.as_float_array(target_orient)))

        
    def get_target_orient(self, mu):
        state                           = self.get_linear_state()
        R_lvlh_to_inertial, omega_hill  = att_ref.get_lvlh_to_inertial(state, mu)
        sat_target_orient               = quaternion.from_rotation_matrix(R_lvlh_to_inertial.T)
        return sat_target_orient
        
    def get_inputs(self, mu):
        state               = self.get_linear_state()
        rotation_state      = self.get_rotational_state()
        sat_target_orient   = self.get_target_orient(mu)

        self.update_target_state_hist(sat_target_orient)
        
        T_body = self.get_thrust_command(state)
        L_body = self.get_torque_command(rotation_state, sat_target_orient)

        return T_body, L_body
        
    def get_thrust_command(self, state):
        return np.array([0, 0, 0])
        
    def get_torque_command(self, rot_state, q_target):
        q_now   = quaternion.as_quat_array(rot_state[0:4])
        omega   = rot_state[4:7]

        # Get the control gains
        k_p = self.orientation_control_gains[0]
        k_d = self.orientation_control_gains[1]

        # Compute the quaternion error
        dq  =   q_now*q_target.inverse()

        # Compute the control input
        u   =   -k_p*np.array([dq.x, dq.y, dq.z]) - k_d*omega
        return u
        