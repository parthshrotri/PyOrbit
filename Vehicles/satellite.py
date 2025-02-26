import yaml
import numpy as np
import quaternion

import utils.convert as convert
import utils.attitude_ref as att_ref
import Vehicles.GNC.control as ctrl
import Vehicles.GNC.sensors as nav

class Satellite():
    def __init__(self, t0, state, central_body, sc_yaml):
        mu                      = central_body.mu
        
        # Get properties from the spacecraft yaml
        sat_props   = yaml.safe_load(open(sc_yaml, 'r'))

        # Set the name
        self.name        = sat_props["name"]

        # Populate the mass/inertia properties
        self.mass                   = float(sat_props["mass"])
        self.J                      = np.diag(sat_props["inertia"])

        # Set the visualization properties
        self.model              = sat_props["model"]
        self.colorscale         = sat_props["colorscale"]
        self.model_axis_order   = sat_props["model_axis_order"]
        self.lights             = np.array([sat_props["nav_red"], 
                                            sat_props["nav_green"]])

        # Set the create the controller
        control_params          = sat_props["control"]
        kp                      = control_params["orient_control_kp"]
        kd                      = control_params["orient_control_kd"]
        orient_control_gains    = [kp, kd]
        self.controller         = ctrl.Controller(orient_control_gains)

        # Create the star tracker
        star_tracker_props  = sat_props["star_tracker"]
        body_to_boresight   = quaternion.from_float_array(star_tracker_props["body_to_boresight"])
        self.star_tracker   = nav.StarTracker(body_to_boresight.conjugate())

        # Initialize the history arrays
        self.init_state             = state
        self.state_history          = np.array([self.init_state])
        self.target_orient_history  = np.array([state[6:10]])

        self.lvlh_to_body_hist      = np.array([quaternion.as_float_array(self.get_lvlh_to_body(mu))])
        self.hill_to_body_hist      = np.array([quaternion.as_float_array(self.get_hill_to_body(mu))])

        t0                          = convert.daysSinceJ2000(t0)
        self.pcpf_hist              = np.array([self.get_pcpf_state(central_body, t0)])
        self.lla_hist               = np.array([self.get_lla(central_body, t0)])

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
    
    def get_pcpf_state(self, planet, days_epoch):
        pci_state       = self.get_linear_state()
        pcpf_state      = att_ref.pci2pcpf(pci_state, planet, days_epoch)
        return pcpf_state

    def get_lla(self, planet, days_epoch):
        pci_state       = self.get_linear_state()
        pcpf_state      = att_ref.pci2pcpf(pci_state, planet, days_epoch)
        lla_state       = att_ref.pcpf2lla(pcpf_state, planet)
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
        
    def update_pcpf_hist(self, planet, days_epoch):
        self.pcpf_hist  = np.vstack((self.pcpf_hist, 
                                     self.get_pcpf_state(planet, days_epoch)))
        
    def update_lla_hist(self, planet, days_epoch):
        self.lla_hist   = np.vstack((self.lla_hist, 
                                     self.get_lla(planet, days_epoch)))
        
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
        q_lvlh_to_inertial, omega_hill  = att_ref.get_lvlh_to_pci(state, mu)
        q_inertial_to_lvlh              = q_lvlh_to_inertial.conjugate()
        return q_inertial_to_lvlh
    
    def get_inertial_to_hill(self, mu):
        state                           = self.get_linear_state()
        q_hill_to_inertial, omega_hill  = att_ref.get_hill_to_pci(state, mu)
        q_inertial_to_hill              = q_hill_to_inertial.conjugate()
        return q_inertial_to_hill

    def get_target_orient(self, mu):
        sat_target_orient               = self.get_inertial_to_lvlh(mu)
        return sat_target_orient
    
    def update_state_hist(self, t0, t_now, state, sat_target_orient, central_body):
        mu              = central_body.mu
        days_epoch      = convert.daysSinceJ2000(t0, t_now)

        self.update_state_history(state)
        self.update_target_state_hist(sat_target_orient)
        self.update_lvlh_to_body_hist(self.get_lvlh_to_body(mu))
        self.update_hill_to_body_hist(self.get_hill_to_body(mu))
        self.update_pcpf_hist(central_body, days_epoch)
        self.update_lla_hist(central_body, days_epoch)
        
    def get_inputs(self, mu):
        state               = self.get_linear_state()
        rotation_state      = self.get_rotational_state()
        sat_target_orient   = self.get_target_orient(mu)
        
        T_body = self.controller.get_thrust_command(state)
        L_body = self.controller.get_torque_cmd(rotation_state, sat_target_orient)

        return sat_target_orient, T_body, L_body
        