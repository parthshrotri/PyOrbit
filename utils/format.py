import numpy as np
from stl import mesh

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

def states_for_plot(sat_pos, sat_orient, quat_targ_history):
    sat_state_plot = np.zeros((len(sat_pos), 11))
    sat_state_plot[:,0:3] = sat_pos
    sat_state_plot[:,3:7] = sat_orient
    sat_state_plot[:,7:11] = quat_targ_history
    return sat_state_plot

# From @empet on plotly forum
def stl2mesh3d(stl_mesh):
    # stl_mesh is read by nympy-stl from a stl file; it is  an array of faces/triangles (i.e. three 3d points) 
    # this function extracts the unique vertices and the lists I, J, K to define a Plotly mesh3d
    p, q, r = stl_mesh.vectors.shape #(p, 3, 3)
    # the array stl_mesh.vectors.reshape(p*q, r) can contain multiple copies of the same vertex;
    # extract unique vertices from all mesh triangles
    vertices, ixr = np.unique(stl_mesh.vectors.reshape(p*q, r), return_inverse=True, axis=0)
    I = np.take(ixr, [3*k for k in range(p)])
    J = np.take(ixr, [3*k+1 for k in range(p)])
    K = np.take(ixr, [3*k+2 for k in range(p)])
    IJK = np.vstack((I, J, K)).T
    return vertices, IJK

def model(satellite):
    # Unpack model location and axis order
    model       = satellite.model
    axis_order  = satellite.model_axis_order
    colorscale  = satellite.colorscale

    # Get model data
    vehicle = mesh.Mesh.from_file(model)
    vehicle_points, IJK = stl2mesh3d(vehicle)

    # Swap the axes to match the vehicle
    vehicle_points = vehicle_points[:, axis_order]
    vehicle_points -= np.mean(vehicle_points, axis=0)

    # Centroid the vehicle
    min_x = np.min(vehicle_points[:,0])
    max_x = np.max(vehicle_points[:,0])
    min_y = np.min(vehicle_points[:,1])
    max_y = np.max(vehicle_points[:,1])
    min_z = np.min(vehicle_points[:,2])
    max_z = np.max(vehicle_points[:,2])

    range_x = max_x - min_x
    range_y = max_y - min_y
    range_z = max_z - min_z

    # Normalize the vehicle scale
    max_range       = max(range_x, range_y, range_z)
    vehicle_points  = vehicle_points/max_range
    vehicle_points  = np.vstack((vehicle_points, satellite.lights)).astype(np.float64)

    return [vehicle_points, IJK, colorscale]