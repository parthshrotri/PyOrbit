import utils.plot as stills
import utils.animation as animate
import utils.format as format

class Visualizer():
    def __init__(self, sim_data, anim_framerate):
        self.sim_data       = sim_data
        self.planet         = sim_data.central_body
        self.satellites     = sim_data.satellites
        self.anim_framerate = anim_framerate

        for i in range(len(self.satellites)):
            sat = self.satellites[i]
            sat.vehicle_model = format.model(sat)

    def run(self, plot):
        sim_data        = self.sim_data
        planet          = sim_data.central_body
        satellites      = sim_data.satellites
        
        planet_letter   = planet.name[0].upper()

        for i in range(len(satellites)):
            sat = self.sim_data.satellites[i]

            if str(f"{planet_letter}ci_anim").casefold() == plot.casefold():
                print(f"Creating {planet_letter}CI Trajectory Animation")
                eci = animate.pci(sat, planet, self.anim_framerate)
                eci.show()

            if "att_inertial_anim" in plot:
                print("Creating Attitude wrt. Inertial Animation")
                att = animate.att_inertial(sat, self.anim_framerate)
                att.show()

            if "att_lvlh_anim" in plot:
                print("Creating Attitude wrt. LVLH Animation")
                lvlh = animate.att_lvlh(sat, self.anim_framerate)
                lvlh.show()

            if "att_hill_anim" in plot:
                print("Creating Attitude wrt. Hill Animation")
                hill = animate.att_hill(sat, self.anim_framerate)
                hill.show()

            if "groundtrack_anim" in plot:
                print("Creating Groundtrack Animation")
                groundtrack = animate.groundtrack(sat, planet, self.anim_framerate)
                groundtrack.show()

            if str("ecef_anim").casefold() == plot.casefold():
                print("Creating ECEF Trajectory Animation")
                ecef = animate.ecef(sat, planet, self.anim_framerate)
                ecef.show()

        if plot == "groundtrack":
            print("Creating Groundtrack Plot")
            groundtrack = stills.ground_track(satellites, planet)
            groundtrack.show()

        if str(f"{planet_letter}ci").casefold() == plot.casefold():
            print(f"Creating {planet_letter}CI Trajectory Plot")
            eci = stills.pci(satellites, planet)
            eci.show()

        if plot == "ecef":
            print("Creating ECEF Trajectory Plot")
            ecef = stills.ecef(satellites, planet)
            ecef.show()