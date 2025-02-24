import numpy as np
import quaternion
import utils.convert as convert
import utils.attitude_ref as att_ref

class Satellite():
    def __init__(self, name, t0, state, mass_prop, visual_prop, mu):
        # Populate the properties
        self.mass                   = mass_prop[0]
        self.J                      = mass_prop[1]
        self.name                   = name

        # Set the visualization properties
        self.model                  = visual_prop[0]
        self.colorscale             = visual_prop[1]
        self.axis_order             = visual_prop[2]
        self.lights                 = visual_prop[3]

        # Set the control gains for attitude control
        self.orientation_control_gains  = np.array([7.0, 35])

        # Initialize the history arrays
        self.init_state             = state
        self.state_history          = np.array([self.init_state])
        self.target_orient_history  = np.array([state[6:10]])

        self.lvlh_to_body_hist      = np.array([quaternion.as_float_array(self.get_lvlh_to_body(mu))])
        self.hill_to_body_hist      = np.array([quaternion.as_float_array(self.get_hill_to_body(mu))])

        t0                          = convert.daysSinceJ2000(t0)
        self.ecef_hist              = np.array([self.get_ecef_state(t0)])
        self.lla_hist               = np.array([self.get_lla(t0)])

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
    
    def get_ecef_state(self, days_epoch):
        eci_state       = self.get_linear_state()
        ecef_state      = att_ref.eci2ecef(eci_state, days_epoch)
        return ecef_state

    def get_lla(self, days_epoch):
        eci_state       = self.get_linear_state()
        ecef_state      = att_ref.eci2ecef(eci_state, days_epoch)
        lla_state       = att_ref.ecef2lla(ecef_state)
        return lla_state
    
    def update_state_history(self, state):
        self.state_history  = np.vstack((self.state_history, state))
    
    def update_target_state_hist(self, target_orient):
        self.target_orient_history  = np.vstack((self.target_orient_history, 
                                                 quaternion.as_float_array(target_orient)))   
        
    def update_lvlh_to_body_hist(self, lvlh_to_body):
        self.lvlh_to_body_hist  = np.vstack((self.lvlh_to_body_hist, 
                                             quaternion.as_float_array(lvlh_to_body)))
        
    def update_hill_to_body_hist(self, hill_to_body):
        self.hill_to_body_hist  = np.vstack((self.hill_to_body_hist, 
                                             quaternion.as_float_array(hill_to_body)))
        
    def update_ecef_hist(self, days_epoch):
        self.ecef_hist  = np.vstack((self.ecef_hist, 
                                     self.get_ecef_state(days_epoch)))
        
    def update_lla_hist(self, days_epoch):
        self.lla_hist   = np.vstack((self.lla_hist, 
                                     self.get_lla(days_epoch)))
        
    def get_lvlh_to_body(self, mu):
        quat                = self.get_quat()
        q_inertial_to_lvlh  = self.get_inertial_to_lvlh(mu)
        q_lvlh_to_body      = quat * q_inertial_to_lvlh.conjugate()
        return q_lvlh_to_body
    
    def get_hill_to_body(self, mu):
        quat                = self.get_quat()
        q_inertial_to_hill  = self.get_inertial_to_hill(mu)
        q_hill_to_body      = quat * q_inertial_to_hill.conjugate()
        return q_hill_to_body

    def get_inertial_to_lvlh(self, mu):
        state                           = self.get_linear_state()
        q_lvlh_to_inertial, omega_hill  = att_ref.get_lvlh_to_inertial(state, mu)
        q_inertial_to_lvlh              = q_lvlh_to_inertial.conjugate()
        return q_inertial_to_lvlh
    
    def get_inertial_to_hill(self, mu):
        state                           = self.get_linear_state()
        q_hill_to_inertial, omega_hill  = att_ref.get_hill_to_inertial(state, mu)
        q_inertial_to_hill              = q_hill_to_inertial.conjugate()
        return q_inertial_to_hill

    def get_target_orient(self, mu):
        sat_target_orient               = self.get_inertial_to_lvlh(mu)
        return sat_target_orient
    
    def update_state_hist(self, t0, t_now, state, sat_target_orient, mu):
        days_epoch      = convert.daysSinceJ2000(t0, t_now)

        self.update_state_history(state)
        self.update_target_state_hist(sat_target_orient)
        self.update_lvlh_to_body_hist(self.get_lvlh_to_body(mu))
        self.update_hill_to_body_hist(self.get_hill_to_body(mu))
        self.update_ecef_hist(days_epoch)
        self.update_lla_hist(days_epoch)
        
    def get_inputs(self, mu):
        state               = self.get_linear_state()
        rotation_state      = self.get_rotational_state()
        sat_target_orient   = self.get_target_orient(mu)
        
        T_body = self.get_thrust_command(state)
        L_body = self.get_torque_command(rotation_state, sat_target_orient)

        return sat_target_orient, T_body, L_body
        
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
        