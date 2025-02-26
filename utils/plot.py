import plotly
import quaternion
import numpy as np
import plotly.graph_objects as go
import utils.format as format

def get_3d_frame_data(plot_states, idx, vehicle_data, scale, draw_thrusters):
    # Unpack input data
    pos         = plot_states[idx, 0:3]
    orient      = plot_states[idx, 3:7]
    quat_targ   = plot_states[idx, 7:11]
    quat        = quaternion.from_float_array(orient)
    quat_targ   = quaternion.from_float_array(quat_targ)

    # Create the spacecraft trajectory
    spacecraft_traj = draw_traj(plot_states[:, 0:3], idx, vehicle_data[2])

    # Plot the spacecraft axes
    sc_axes    = draw_spacecraft_axes(pos, quat, scale)
    
    # Plot the target axes
    targ_axes   = draw_target_axes(pos, quat_targ, scale)
    
    # Plot the spacecraft
    spacecraft = draw_spacecraft(pos, quat, vehicle_data, scale, draw_thrusters)
    
    # Add all the objects to the list
    objs = spacecraft_traj
    objs.extend(targ_axes)
    objs.extend(sc_axes)
    objs.extend(spacecraft)
    return objs

def draw_traj(posits, idx, colorscale):
    spacecraft_traj  = go.Scatter3d(x=posits[0:idx,0],
                                    y=posits[0:idx,1],
                                    z=posits[0:idx,2],
                                    mode="lines",
                                    line=dict(width=5, colorscale=colorscale, 
                                              color=np.arange(0, idx, 1), 
                                              showscale=False), showlegend=False)
    return [spacecraft_traj]

def draw_spacecraft_axes(pos, orient, scale):
    # Convert the quaternion to a rotation matrix to get the axes
    rot         = quaternion.as_rotation_matrix(orient)

    # Create the data for the plot
    # Plot the spacecraft axes
    x_x_comps   = [pos[0], pos[0] + rot[0,0]*scale]
    x_y_comps   = [pos[1], pos[1] + rot[0,1]*scale]
    x_z_comps   = [pos[2], pos[2] + rot[0,2]*scale]
    x_axis_obj  = go.Scatter3d(x=x_x_comps,
                               y=x_y_comps,
                               z=x_z_comps,
                               mode="lines", 
                               line=dict(width=3, color="red"), showlegend=False)
    
    y_x_comps   = [pos[0], pos[0] + rot[1,0]*scale]
    y_y_comps   = [pos[1], pos[1] + rot[1,1]*scale]
    y_z_comps   = [pos[2], pos[2] + rot[1,2]*scale]
    y_axis_obj  = go.Scatter3d(x=y_x_comps,
                               y=y_y_comps,
                               z=y_z_comps,
                               mode="lines", 
                               line=dict(width=3, color="green"), showlegend=False)
    
    z_x_comps   = [pos[0], pos[0] + rot[2,0]*scale]
    z_y_comps   = [pos[1], pos[1] + rot[2,1]*scale]
    z_z_comps   = [pos[2], pos[2] + rot[2,2]*scale]
    z_axis_obj  = go.Scatter3d(x=z_x_comps,
                               y=z_y_comps,
                               z=z_z_comps,
                               mode="lines", 
                               line=dict(width=3, color="blue"), showlegend=False)
    return [x_axis_obj, y_axis_obj, z_axis_obj]

def draw_target_axes(pos, orient, scale):
    rot_targ    = quaternion.as_rotation_matrix(orient)

    # Plot the target axes
    x_target_x_comps    = [pos[0], pos[0] + rot_targ[0,0]*scale]
    x_target_y_comps    = [pos[1], pos[1] + rot_targ[0,1]*scale]
    x_target_z_comps    = [pos[2], pos[2] + rot_targ[0,2]*scale]
    x_target_obj        = go.Scatter3d(x=x_target_x_comps,
                                       y=x_target_y_comps,
                                       z=x_target_z_comps,
                                       mode="lines", 
                                       line=dict(width=3, color="purple"),
                                       showlegend=False)
    
    y_target_x_comps    = [pos[0], pos[0] + rot_targ[1,0]*scale]
    y_target_y_comps    = [pos[1], pos[1] + rot_targ[1,1]*scale]
    y_target_z_comps    = [pos[2], pos[2] + rot_targ[1,2]*scale]
    y_target_obj        = go.Scatter3d(x=y_target_x_comps,
                                       y=y_target_y_comps,
                                       z=y_target_z_comps,
                                       mode="lines", 
                                       line=dict(width=3, color="purple"),
                                       showlegend=False)
    
    z_target_x_comps    = [pos[0], pos[0] + rot_targ[2,0]*scale]
    z_target_y_comps    = [pos[1], pos[1] + rot_targ[2,1]*scale]
    z_target_z_comps    = [pos[2], pos[2] + rot_targ[2,2]*scale]
    z_target_obj        = go.Scatter3d(x=z_target_x_comps,
                                       y=z_target_y_comps,
                                       z=z_target_z_comps,
                                       mode="lines", 
                                       line=dict(width=3, color="purple"),
                                       showlegend=False)
    
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
                                  showscale = False, showlegend=False)
    
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
                                  showscale = False, showlegend=False)
                          
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
                                  showscale = False, showlegend=False)
    
    return [x_target_obj, y_target_obj, z_target_obj, x_targ_cone, y_targ_cone, z_targ_cone]

def draw_spacecraft(pos, orient, vehicle_data, scale, plot_thrusters):
    # Unpack input data    
    sc_scaled   = vehicle_data[0]*scale
    vertices    = vehicle_data[1]

    # Convert the quaternion to a rotation matrix to get the axes
    rot         = quaternion.as_rotation_matrix(orient)

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
                                lighting=dict(diffuse=.2), showlegend=False)
    
    red_light      = go.Scatter3d(x=[spacecraft_rot[-2,0]],
                                  y=[spacecraft_rot[-2,1]],
                                  z=[spacecraft_rot[-2,2]],
                                  mode="markers",
                                  marker=dict(size=2, color="red"), showlegend=False)
    
    green_light    = go.Scatter3d(x=[spacecraft_rot[-1,0]],
                                  y=[spacecraft_rot[-1,1]],
                                  z=[spacecraft_rot[-1,2]],
                                  mode="markers",
                                  marker=dict(size=2, color="green"), showlegend=False)

    # Add all the objects to the list
    objs    = [spacecraft, red_light, green_light]
    
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

def pci(satellites, body): 
    scale = 350000.0 
    objs = [draw_planet(body)]
    max_global_range = 0
    for sc in satellites:
        sat_pos             = sc.state_history[:,0:3]
        sat_orient          = sc.state_history[:,6:10]
        quat_targ_history   = sc.target_orient_history
        vehicle_model       = sc.vehicle_model

        sat_state_plot  = format.states_for_plot(sat_pos, sat_orient, quat_targ_history)
        draw_thrusters  = False
        frame           = get_3d_frame_data(sat_state_plot, 
                                            len(sat_state_plot)-1, 
                                            vehicle_model, scale, 
                                            draw_thrusters)
        objs.extend(frame)

        max_range = np.max(np.abs(sc.state_history[:,0:3]))
        if max_range > max_global_range:
            max_global_range = max_range

    min_axis = -1.2*max_global_range
    max_axis = 1.2*max_global_range
    
    xaxis   = dict(range=[min_axis, max_axis])
    yaxis   = dict(range=[min_axis, max_axis])
    zaxis   = dict(range=[min_axis, max_axis])

    fig = go.Figure(data=objs)
    fig.update_layout(title=f"{body.name[0]}CI Trajectory",scene = dict(xaxis_title='X [m]',
                                                          yaxis_title='Y [m]',
                                                          zaxis_title='Z [m]',
                                                            aspectmode="cube", 
                                                            xaxis=xaxis, 
                                                            yaxis=yaxis, 
                                                            zaxis=zaxis),
                                                font=dict(family="Courier New, monospace",
                                                          size=18,color="RebeccaPurple"))
    return fig

def create_ground_circle(radius, R, center_lat, center_lon):
    angles = np.linspace(0, 2*np.pi, 100)

    lat_rad = np.radians(center_lat)
    lon_rad = np.radians(center_lon)

    lat_points = np.arcsin(np.sin(lat_rad) * np.cos(radius/R) + 
                           np.cos(lat_rad) * np.sin(radius/R) * np.cos(angles))
    lon_points = lon_rad + np.arctan2(np.sin(angles) * np.sin(radius/R) * np.cos(lat_rad),
                                     np.cos(radius/R) - np.sin(lat_rad) * np.sin(lat_points))
    
    lat_points = np.degrees(lat_points)
    lon_points = np.degrees(lon_points)

    circle = go.Scattergeo(lat=lat_points, lon=lon_points, mode="lines", 
                           line=dict(width=2, color="whitesmoke"), 
                           showlegend=False)
    return circle

def ground_track_frame(sc, body, idx):
    objs =[]
    state_lla_hist  = sc.lla_hist
    R_body          = body.radius
    alt             = state_lla_hist[idx,2]
    angle           = np.arccos(R_body/(R_body + alt))
    arc_len         = R_body*angle
    num_points      = len(state_lla_hist[0:idx,0])
    vis_range       = create_ground_circle(arc_len, R_body, state_lla_hist[idx,0], state_lla_hist[idx,1])

    traj            = go.Scattergeo(lat=state_lla_hist[0:idx,0], lon=state_lla_hist[0:idx,1], 
                                mode="markers", marker=dict(size=4, color=np.arange(num_points), 
                                                            colorscale=sc.colorscale), showlegend=False)
    sc_icon_color   = plotly.colors.get_colorscale(sc.colorscale)[-1]
    last_pos        = go.Scattergeo(lat=[state_lla_hist[idx,0]], lon=[state_lla_hist[idx,1]],
                                mode="markers", marker=dict(size=15, color=sc_icon_color, 
                                                            symbol = "triangle-right"), 
                                name=sc.name)
    objs.extend([vis_range, traj, last_pos])
    return objs

def ground_track(spacecrafts, body): 
    fig     = go.Figure();  
    for sc in spacecrafts:
        frame   = ground_track_frame(sc, body, -1)
        fig.add_traces(frame)
        
    fig.update_layout(title="Ground Track",
                      font=dict(family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"))
    fig.update_geos(projection_type = "equirectangular",
                    showland=True, 
                    coastlinewidth=2,
                    landcolor="rgb(52, 165, 111)",
                    showocean=True,
                    oceancolor="rgb(0, 204, 255)",
                    showcountries=True, countrycolor="darkgreen",
                    showlakes=True, lakecolor="rgb(0, 130, 255)",
                    showrivers=True, rivercolor="rgb(0, 130, 255)",
                    lonaxis=dict(range=[-180, 180], dtick = 20, showgrid=True, gridcolor="grey"),
                    lataxis=dict(range=[-90, 90], dtick = 20, showgrid=True, gridcolor="grey"),
                    resolution=50)
    return fig

def ecef(satellites, body):
    ecef_fig = ground_track(satellites, body)
    ecef_fig.update_layout(title=f"ECEF Trajectory")
    ecef_fig.update_geos(lonaxis=dict(range=[-180, 180], dtick = 20, showgrid=True, gridcolor="grey"),
                         lataxis=dict(range=[-180, 180], dtick = 20, showgrid=True, gridcolor="grey"))
    ecef_fig.update_geos(projection_type = "orthographic")
    return ecef_fig