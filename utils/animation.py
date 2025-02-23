# Import necessary libraries
import numpy as np
import quaternion
import plotly.graph_objects as go
from stl import mesh
from tqdm import tqdm

# Plotting utility to generate the data for each frame of the animation
def get_frame_data(posits, orients, idx, quat_targ, vehicle_data, scale):
    # Unpack input data
    pos         = posits[idx, 0:3]
    quat        = np.quaternion(orients[idx, 0], 
                                orients[idx, 1], 
                                orients[idx, 2], 
                                orients[idx ,3])

    quat_targ   = np.quaternion(quat_targ[0], 
                                quat_targ[1], 
                                quat_targ[2], 
                                quat_targ[3])
    
    sc_scaled   = vehicle_data[0]*scale
    vertices    = vehicle_data[1]

    # Convert the quaternion to a rotation matrix to get the axes
    rot         = quaternion.as_rotation_matrix(quat)
    rot_targ    = quaternion.as_rotation_matrix(quat_targ)

    # Create the data for the plot
    # Plot the spacecraft axes
    x_x_comps   = [pos[0], pos[0] + rot[0,0]*scale]
    x_y_comps   = [pos[1], pos[1] + rot[0,1]*scale]
    x_z_comps   = [pos[2], pos[2] + rot[0,2]*scale]
    x_axis_obj  = go.Scatter3d(x=x_x_comps,
                               y=x_y_comps,
                               z=x_z_comps,
                               mode="lines", 
                               line=dict(width=3, color="red"))
    
    y_x_comps   = [pos[0], pos[0] + rot[1,0]*scale]
    y_y_comps   = [pos[1], pos[1] + rot[1,1]*scale]
    y_z_comps   = [pos[2], pos[2] + rot[1,2]*scale]
    y_axis_obj  = go.Scatter3d(x=y_x_comps,
                               y=y_y_comps,
                               z=y_z_comps,
                               mode="lines", 
                               line=dict(width=3, color="green"))
    
    z_x_comps   = [pos[0], pos[0] + rot[2,0]*scale]
    z_y_comps   = [pos[1], pos[1] + rot[2,1]*scale]
    z_z_comps   = [pos[2], pos[2] + rot[2,2]*scale]
    z_axis_obj  = go.Scatter3d(x=z_x_comps,
                               y=z_y_comps,
                               z=z_z_comps,
                               mode="lines", 
                               line=dict(width=3, color="blue"))
    
    # Plot the target axes
    x_target_x_comps    = [pos[0], pos[0] + rot_targ[0,0]*scale]
    x_target_y_comps    = [pos[1], pos[1] + rot_targ[0,1]*scale]
    x_target_z_comps    = [pos[2], pos[2] + rot_targ[0,2]*scale]
    x_target_obj        = go.Scatter3d(x=x_target_x_comps,
                                       y=x_target_y_comps,
                                       z=x_target_z_comps,
                                       mode="lines", 
                                       line=dict(width=3, color="purple"))
    
    y_target_x_comps    = [pos[0], pos[0] + rot_targ[1,0]*scale]
    y_target_y_comps    = [pos[1], pos[1] + rot_targ[1,1]*scale]
    y_target_z_comps    = [pos[2], pos[2] + rot_targ[1,2]*scale]
    y_target_obj        = go.Scatter3d(x=y_target_x_comps,
                                       y=y_target_y_comps,
                                       z=y_target_z_comps,
                                       mode="lines", 
                                       line=dict(width=3, color="purple"))
    
    z_target_x_comps    = [pos[0], pos[0] + rot_targ[2,0]*scale]
    z_target_y_comps    = [pos[1], pos[1] + rot_targ[2,1]*scale]
    z_target_z_comps    = [pos[2], pos[2] + rot_targ[2,2]*scale]
    z_target_obj        = go.Scatter3d(x=z_target_x_comps,
                                       y=z_target_y_comps,
                                       z=z_target_z_comps,
                                       mode="lines", 
                                       line=dict(width=3, color="purple"))
    
    # Plot cones at the ends of the target axes
    x_target_cone_x     = [pos[0] + rot_targ[0,0]*scale]
    x_target_cone_y     = [pos[1] + rot_targ[0,1]*scale]
    x_target_cone_z     = [pos[2] + rot_targ[0,2]*scale]
    x_target_cone_u     = [rot_targ[0,0]*scale*.3]
    x_target_cone_v     = [rot_targ[0,1]*scale*.3]
    x_target_cone_w     = [rot_targ[0,2]*scale*.3]
    x_targ_cone         = go.Cone(x=x_target_cone_x,
                                  y=x_target_cone_y,
                                  z=x_target_cone_z,
                                  u=x_target_cone_u,
                                  v=x_target_cone_v,
                                  w=x_target_cone_w,
                                  anchor = "tail",
                                  hoverinfo =  None,
                                  colorscale = [[0, "red"], [1, "red"]],
                                  showscale = False)
    
    y_target_cone_x     = [pos[0] + rot_targ[1,0]*scale]
    y_target_cone_y     = [pos[1] + rot_targ[1,1]*scale]
    y_target_cone_z     = [pos[2] + rot_targ[1,2]*scale]
    y_target_cone_u     = [rot_targ[1,0]*scale*.3]
    y_target_cone_v     = [rot_targ[1,1]*scale*.3]
    y_target_cone_w     = [rot_targ[1,2]*scale*.3]
    y_targ_cone         = go.Cone(x=y_target_cone_x,
                                  y=y_target_cone_y,
                                  z=y_target_cone_z,
                                  u=y_target_cone_u,
                                  v=y_target_cone_v,
                                  w=y_target_cone_w,
                                  anchor = "tail",
                                  hoverinfo =  None,
                                  colorscale = [[0, "green"], [1, "green"]],
                                  showscale = False)
                          
    z_targ_cone_x       = [pos[0] + rot_targ[2,0]*scale]
    z_targ_cone_y       = [pos[1] + rot_targ[2,1]*scale]
    z_targ_cone_z       = [pos[2] + rot_targ[2,2]*scale]
    z_targ_cone_u       = [rot_targ[2,0]*scale*.3]
    z_targ_cone_v       = [rot_targ[2,1]*scale*.3]
    z_targ_cone_w       = [rot_targ[2,2]*scale*.3]
    z_targ_cone         = go.Cone(x=z_targ_cone_x,
                                  y=z_targ_cone_y,
                                  z=z_targ_cone_z,
                                  u=z_targ_cone_u,
                                  v=z_targ_cone_v,
                                  w=z_targ_cone_w,
                                  anchor = "tail",
                                  hoverinfo =  None,
                                  colorscale = [[0, "blue"], [1, "blue"]],
                                  showscale = False)
    
    # Plot the spacecraft Trajectory
    spacecraft_traj  = go.Scatter3d(x=posits[0:idx,0],
                                    y=posits[0:idx,1],
                                    z=posits[0:idx,2],
                                    mode="lines",
                                    line=dict(width=5, 
                                              colorscale="Oranges", 
                                              color=np.arange(0, idx, 1), 
                                              showscale=False))
    
    # Rotate and translate points of the spacecraft model
    spacecraft_rot   = np.dot(sc_scaled, rot) + pos

    # Plot the spacecraft
    spacecraft      = go.Mesh3d(x=spacecraft_rot[:,0],
                                y=spacecraft_rot[:,1],
                                z=spacecraft_rot[:,2],
                                i=vertices[:,0],
                                j=vertices[:,1],
                                k=vertices[:,2],
                                alphahull=0,
                                opacity=1.00,
                                color="snow", 
                                lighting=dict(diffuse=.2))
    
    red_light      = go.Scatter3d(x=[spacecraft_rot[-2,0]],
                                  y=[spacecraft_rot[-2,1]],
                                  z=[spacecraft_rot[-2,2]],
                                  mode="markers",
                                  marker=dict(size=2, color="red"))
    
    green_light    = go.Scatter3d(x=[spacecraft_rot[-1,0]],
                                  y=[spacecraft_rot[-1,1]],
                                  z=[spacecraft_rot[-1,2]],
                                  mode="markers",
                                  marker=dict(size=2, color="green"))

    # Add all the objects to the list
    objs =[spacecraft_traj,
            x_axis_obj,
            y_axis_obj,
            z_axis_obj,
            x_target_obj,
            y_target_obj,
            z_target_obj,
            x_targ_cone, 
            y_targ_cone, 
            z_targ_cone,
            spacecraft, 
            red_light, 
            green_light]
    
    return objs

def draw_planet(planet):
    # Polar coordinates
    theta   = np.linspace(0, 2.*np.pi, 40)
    phi     = np.linspace(0, np.pi, 40)

    # Convert to cartesian
    x       = planet.radius * np.outer(np.cos(theta), np.sin(phi))
    y       = planet.radius * np.outer(np.sin(theta), np.sin(phi))
    z       = planet.radius * np.outer(np.ones(np.size(theta)), np.cos(phi))

    # Create the planet object
    planet  = go.Surface(x=x, y=y, z=z, opacity=1.0, colorscale=planet.colors, showscale=False)
    return planet

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

def format_model(satellite):
    # Unpack model location and axis order
    model       = satellite.model
    axis_order  = satellite.axis_order

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

    return vehicle_points, IJK

def create_animation(fig, plotformat, sat_pos, sat_orient, quat_targ_history, vehicle_model, update_rate):
    title       = plotformat[0]
    scale       = plotformat[1]
    xaxis       = plotformat[2]
    yaxis       = plotformat[3]
    zaxis       = plotformat[4]
    
    vehicle_points, IJK = vehicle_model

    # Get the data for each frame
    fig.update(frames=[go.Frame(data=[*get_frame_data(sat_pos, sat_orient, i*update_rate, 
                                                      quat_targ_history[i*update_rate], 
                                                      (vehicle_points, IJK), scale)], 
                                traces=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]) 
                                for i in tqdm(range(1, round(len(sat_pos)/update_rate)))])
    
    fig.update_layout(title=dict(text=title),
                      transition = {'duration': 1},
                      scene=dict(aspectmode="cube",
                                 xaxis=xaxis,
                                 yaxis=yaxis,
                                 zaxis=zaxis),
                                 updatemenus=[dict(type="buttons",
                                                   buttons=[dict(label="Play",
                                                                 method="animate",
                                                                 args=[None, {"frame": {"duration": 1, "redraw": True}}])])],
                        showlegend=False)
    return fig


def att_inertial(title, satellite, update_rate):
    scale       = 1
    xaxis       = dict(range=[-1.2, 1.2])
    yaxis       = dict(range=[-1.2, 1.2])
    zaxis       = dict(range=[-1.2, 1.2])
    plotformat  = [title, scale, xaxis, yaxis, zaxis]

    vehicle_model = format_model(satellite)

    sat_pos             = np.zeros_like(satellite.state_history[:,0:3])
    sat_orient          = satellite.state_history[:,6:10]
    quat_targ_history   = satellite.target_orient_history

    frame_data  = get_frame_data(sat_pos, sat_orient, 0, quat_targ_history[0], vehicle_model, scale)
    fig         = go.Figure(data=frame_data)

    animation = create_animation(fig, plotformat, sat_pos, sat_orient, quat_targ_history, vehicle_model, update_rate)
    return animation

def pci(title, satellite, planet, update_rate):
    scale   = 350000.0
    xaxis   = dict(range=[-9000e3, 9000e3])
    yaxis   = dict(range=[-9000e3, 9000e3])
    zaxis   = dict(range=[-9000e3, 9000e3])
    plotformat = [title, scale, xaxis, yaxis, zaxis]
    
    vehicle_model = format_model(satellite)

    sat_pos             = satellite.state_history[:,0:3]
    sat_orient          = satellite.state_history[:,6:10]
    quat_targ_history   = satellite.target_orient_history

    frame_data  = get_frame_data(sat_pos, sat_orient, 0, quat_targ_history[0], vehicle_model, scale)
    frame_data.append(draw_planet(planet))
    fig         = go.Figure(data=frame_data)

    animation = create_animation(fig, plotformat, sat_pos, sat_orient, quat_targ_history, vehicle_model, update_rate)
    return animation