import numpy as np
import utils.animation as animate

sim_dat = np.load("output/sim.npy", allow_pickle=True).item()
planet  = sim_dat.central_body

for i in range(len(sim_dat.satellites)):
    sat = sim_dat.satellites[i]

    eci = animate.pci("ECI Trajectory", sat, planet, 45)
    att = animate.att_inertial("Attitude in Inertial Frame", sat, 45)

    eci.show()
    att.show()