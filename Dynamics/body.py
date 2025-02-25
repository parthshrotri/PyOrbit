class Body:
    def __init__(self, name, planet_data, warn=False):
        self.name   = name

        self.radius         = float(planet_data["radius"])
        self.mu             = float(planet_data["mu"])
        self.J2             = float(planet_data["J2"])
        self.year_length    = float(planet_data["year_length"])
        self.sidereal_day   = float(planet_data["sidereal_day"])
        self.colors         = planet_data["colors"]

        if "atmosphere" in planet_data.keys():
            planet_atmosphere   = planet_data["atmosphere"]
            self.rho0           = float(planet_atmosphere["rho"])
            self.scaleHeight    = float(planet_atmosphere["scaleHeight"])
        else:
            self.rho0           = 0
            self.scaleHeight    = None
            print(f"Warning: Atmosphere not supported for {self.name}")
