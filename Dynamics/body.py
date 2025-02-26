import numpy as np
class Body:
    def __init__(self, name, planet_data):
        self.name   = name

        self.radius         = float(planet_data["radius"])
        self.polar_radius   = float(planet_data["polar_radius"])
        self.mu             = float(planet_data["mu"])
        self.J2             = float(planet_data["J2"])
        self.year_length    = float(planet_data["year_length"])
        self.sidereal_day   = float(planet_data["sidereal_day"])
        self.obliquity      = float(planet_data["obliquity"])
        self.RA_NP          = np.radians(float(planet_data["RA_NPJ2000"]))
        self.DEC_NP         = np.radians(float(planet_data["DEC_NPJ2000"]))
        self.rot_offset     = float(planet_data["rot_offset"])
        self.colors         = planet_data["colors"]

        self.ax_tilt_icrs  = np.array([np.cos(self.DEC_NP)*np.cos(self.RA_NP),
                                        np.cos(self.DEC_NP)*np.sin(self.RA_NP),
                                        np.sin(self.DEC_NP)])

        if "atmosphere" in planet_data.keys():
            planet_atmosphere   = planet_data["atmosphere"]
            self.rho0           = float(planet_atmosphere["rho"])
            self.scaleHeight    = float(planet_atmosphere["scaleHeight"])
        else:
            self.rho0           = 0
            self.scaleHeight    = None
            print(f"Warning: Atmosphere not supported for {self.name}")
