import numpy as np
import quaternion
from tqdm import tqdm
from scipy.integrate import solve_ivp

import utils.attitude_ref as att_ref
import Dynamics.propagation as prop

class Simulator():
    def __init__(self, central_body, tf, dt, satellites, save_file):
        self.central_body   = central_body
        self.tf             = tf
        self.dt             = dt
        self.time_array     = np.arange(0, tf+dt, dt)
        self.satellites     = satellites
        self.save_file      = save_file

    def tic(self, t_now, t_next):
        for sat in self.satellites:
            T_body, L_body = sat.get_inputs(self.central_body.mu)
            
            sol = solve_ivp(prop.state_dot, t_span=[t_now, t_next], y0=sat.get_state(), 
                            args=(sat.mass, sat.J, T_body, L_body, self.central_body.radius, 
                                  self.central_body.mu, self.central_body.J2), 
                            method='RK45')
            
            sat.update_state_history(sol.y[:,-1])

    def save(self):
        np.save(self.save_file, self)
    
    def run(self):
        for i in tqdm(range(len(self.time_array)-1)):
            t_now   = self.time_array[i]
            t_next  = self.time_array[i+1]
            self.tic(t_now, t_next)
        
        self.save()
            
