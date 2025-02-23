import numpy as np
import plotly.graph_objects as go
import utils.convert as convert

def state_vec(name, state, sigfigs=6):
    print(f"{name} State Vector:")
    print(f"R: [{state[0]:.{sigfigs}}, {state[1]:.{sigfigs}}, {state[2]:.{sigfigs}}] m")
    print(f"V: [{state[3]:.{sigfigs}}, {state[4]:.{sigfigs}}, {state[5]:.{sigfigs}}] m/s\n")

def kep_elem(name, kep, sigfigs=6):
    print(f"{name} Keplerian Elements:")
    print(f"Semimajor Axis (a): \t{kep[0]/1e3:.{sigfigs}} km")
    print(f"Eccentricity (e): \t{kep[1]:.{sigfigs}}")
    print(f"Inclination (i): \t{np.degrees(kep[2]):.{sigfigs}}º")
    print(f"RAAN (Ω): \t\t{np.degrees(kep[3]):.{sigfigs}}º")
    print(f"Arg Periapse (𝜔): \t{np.degrees(kep[4]):.{sigfigs}}º")
    print(f"True Anomaly (f): \t{np.degrees(kep[5]):.{sigfigs}}º\n")