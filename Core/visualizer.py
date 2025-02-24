import numpy as np
import utils.plot as plot
import utils.animation as animate

class Visualizer():
    def __init__(self, sim_data, anim_framerate, groundmap=None):
        self.sim_data       = sim_data
        self.anim_framerate = anim_framerate

    def run(self, plots):
        sim_data = self.sim_data
        planet  = sim_data.central_body

        for i in range(len(self.sim_data.satellites)):
            sat = self.sim_data.satellites[i]

            if "eci_anim" in plots:
                print("Creating ECI Trajectory Animation")
                eci = animate.eci("ECI Trajectory", sat, planet, self.anim_framerate)
                eci.show()

            if "ecef_anim" in plots:
                print("Creating ECEF Trajectory Animation")
                ecef = animate.ecef("ECEF Trajectory", sat, planet, self.anim_framerate)
                ecef.show()

            if "att_inertial_anim" in plots:
                print("Creating Attitude wrt. Inertial Animation")
                att = animate.att_inertial("Attitude in Inertial Frame", sat, self.anim_framerate)
                att.show()

            if "att_lvlh_anim" in plots:
                print("Creating Attitude wrt. LVLH Animation")
                lvlh = animate.att_lvlh("Attitude wrt LVLH Frame", sat, self.anim_framerate)
                lvlh.show()

            if "att_hill_anim" in plots:
                print("Creating Attitude wrt. Hill Animation")
                hill = animate.att_hill("Attitude wrt Hill Frame", sat, self.anim_framerate)
                hill.show()

        if "groundtrack" in plots:
            print("Creating Groundtrack Plot")
            groundtrack = plot.ground_track(self.sim_data.satellites)
            groundtrack.show()

        if "eci" in plots:
            print("Creating ECI Trajectory Plot")
            eci = plot.eci(self.sim_data.satellites, planet)
            eci.show()

        if "ecef" in plots:
            print("Creating ECEF Trajectory Plot")
            eci = plot.ecef(self.sim_data.satellites)
            eci.show()