import numpy as np
from tqdm import tqdm
from scipy.integrate import solve_ivp

import Dynamics.propagation as prop

class Simulator():
    def __init__(self, central_body, t0, tf, dt, satellites, save_file):
        self.central_body   = central_body
        self.t0             = t0
        self.t_now          = 0
        self.tf             = tf
        self.dt             = dt
        self.time_array     = np.arange(0, tf+dt, dt)
        self.satellites     = satellites
        self.save_file      = save_file

    def tic(self, t_now, t_next):
        for sat in self.satellites:
            q_target, T_body, L_body = sat.get_inputs(self.central_body.mu)
            
            sol = solve_ivp(prop.state_dot, t_span=[t_now, t_next], y0=sat.get_state(), 
                            args=(sat.mass, sat.J, T_body, L_body, self.central_body), 
                            method='RK45')
            
            sat.update_state_hist(self.t0, self.t_now, sol.y[:,-1,], q_target, self.central_body)

    def save(self):
        np.save(self.save_file, self)
    
    def run(self):
        for i in tqdm(range(len(self.time_array)-1)):
            self.t_now = self.time_array[i]
            self.tic(self.t_now, self.t_now + self.dt)
        
        self.save()
            
