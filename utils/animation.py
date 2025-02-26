# Import necessary libraries
import numpy as np
import plotly.graph_objects as go

from tqdm import tqdm

import utils.plot as still
import utils.format as format

def create_groundtrack_animation(fig, satellite, planet, update_rate):
    total_states = len(satellite.state_history)
    # Get the data for each frame
    fig.update(frames=[go.Frame(data=[*still.ground_track_frame(satellite, planet, i*update_rate)], 
                                traces=[0, 1, 2, 3]) 
                                for i in tqdm(range(1, round(total_states/update_rate)))])
    
    fig.update_layout(title=dict(text="Ground Track"),
                      font=dict(family="Courier New, monospace",
                                size=18,color="RebeccaPurple"),
                      transition = {'duration': 1},
                      updatemenus=[dict(type="buttons",buttons=[dict(label="Play",method="animate",
                                                args=[None, {"frame": {"duration": 1, "redraw": True}}])])])
    return fig

def create_3d_animation(fig, plotformat, sat_state_plot, vehicle_model, draw_thrusters, update_rate):
    # Unpack the plot format
    title       = plotformat[0]
    scale       = plotformat[1]
    xaxis       = plotformat[2]
    yaxis       = plotformat[3]
    zaxis       = plotformat[4]

    # Get the data for each frame
    fig.update(frames=[go.Frame(data=[*still.get_3d_frame_data(sat_state_plot, i*update_rate,
                                                      vehicle_model, scale, draw_thrusters)], 
                                traces=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]) 
                                for i in tqdm(range(1, round(len(sat_state_plot)/update_rate)))])
    
    fig.update_layout(title=dict(text=title),
                      font=dict(family="Courier New, monospace",
                                size=18,color="RebeccaPurple"),
                      transition = {'duration': 1},
                      scene=dict(aspectmode="cube",
                                 xaxis=xaxis,yaxis=yaxis,zaxis=zaxis),
                                 updatemenus=[dict(type="buttons",buttons=[dict(label="Play",method="animate",
                                                args=[None, {"frame": {"duration": 1, "redraw": True}}])])],
                        showlegend=False)
    return fig

def att_hill(satellite, update_rate):
    scale       = 1
    xaxis       = dict(range=[-1.2, 1.2])
    yaxis       = dict(range=[-1.2, 1.2])
    zaxis       = dict(range=[-1.2, 1.2])
    plotformat  = ["Attitude wrt. Hill Frame", scale, xaxis, yaxis, zaxis]

    sat_pos                 = np.zeros_like(satellite.state_history[:,0:3])
    sat_orient              = satellite.hill_to_body_hist
    quat_targ_history       = np.zeros((len(satellite.state_history), 4))
    quat_targ_history[:,0]  = 1
    vehicle_model           = satellite.vehicle_model

    sat_state_plot  = format.states_for_plot(sat_pos, sat_orient, quat_targ_history)
    plot_thrusters  = True

    frame_data  = still.get_3d_frame_data(sat_state_plot, 0, vehicle_model, scale, plot_thrusters)
    fig         = go.Figure(data=frame_data)
    
    animation   = create_3d_animation(fig, plotformat, sat_state_plot, vehicle_model, plot_thrusters, update_rate)
    return animation

def att_lvlh(satellite, update_rate):
    scale       = 1
    xaxis       = dict(range=[-1.2, 1.2])
    yaxis       = dict(range=[-1.2, 1.2])
    zaxis       = dict(range=[-1.2, 1.2])
    plotformat  = ["Attitude wrt. LVLH", scale, xaxis, yaxis, zaxis]

    sat_pos                 = np.zeros_like(satellite.state_history[:,0:3])
    sat_orient              = satellite.lvlh_to_body_hist
    quat_targ_history       = np.zeros((len(satellite.state_history), 4))
    quat_targ_history[:,0]  = 1
    vehicle_model           = satellite.vehicle_model

    sat_state_plot  = format.states_for_plot(sat_pos, sat_orient, quat_targ_history)
    plot_thrusters  = True

    frame_data  = still.get_3d_frame_data(sat_state_plot, 0, vehicle_model, scale, plot_thrusters)
    fig         = go.Figure(data=frame_data)

    animation   = create_3d_animation(fig, plotformat, sat_state_plot, vehicle_model, plot_thrusters, update_rate)
    return animation

def att_inertial(satellite, update_rate):
    scale       = 1
    xaxis       = dict(range=[-1.2, 1.2])
    yaxis       = dict(range=[-1.2, 1.2])
    zaxis       = dict(range=[-1.2, 1.2])
    plotformat  = ["Attitude wrt. Inertial", scale, xaxis, yaxis, zaxis]

    sat_pos             = np.zeros_like(satellite.state_history[:,0:3])
    sat_orient          = satellite.state_history[:,6:10]
    quat_targ_history   = satellite.target_orient_history
    vehicle_model       = satellite.vehicle_model

    sat_state_plot  = format.states_for_plot(sat_pos, sat_orient, quat_targ_history)
    plot_thrusters  = True

    frame_data  = still.get_3d_frame_data(sat_state_plot, 0, vehicle_model, scale, plot_thrusters)
    fig         = go.Figure(data=frame_data)

    animation   = create_3d_animation(fig, plotformat, sat_state_plot, vehicle_model, plot_thrusters, update_rate)
    return animation

def pci(satellite, planet, update_rate):
    scale   = 350000.0

    max_range = np.max(np.abs(satellite.state_history[:,0:3]))

    min_axis = -1.2*max_range
    max_axis = 1.2*max_range

    xaxis   = dict(range=[min_axis, max_axis])
    yaxis   = dict(range=[min_axis, max_axis])
    zaxis   = dict(range=[min_axis, max_axis])
    plotformat = [f"{planet.name[0]}CI Trajectory", scale, xaxis, yaxis, zaxis]

    sat_pos             = satellite.state_history[:,0:3]
    sat_orient          = satellite.state_history[:,6:10]
    quat_targ_history   = satellite.target_orient_history
    vehicle_model       = satellite.vehicle_model

    sat_state_plot  = format.states_for_plot(sat_pos, sat_orient, quat_targ_history)
    plot_thrusters  = False

    frame_data  = still.get_3d_frame_data(sat_state_plot, 0, vehicle_model, scale, plot_thrusters)
    frame_data.append(still.draw_planet(planet))
    fig         = go.Figure(data=frame_data)

    animation = create_3d_animation(fig, plotformat, sat_state_plot, vehicle_model, plot_thrusters, update_rate)
    return animation

def groundtrack(satellite, planet, update_rate):
    frame_data  = still.ground_track_frame(satellite, planet, 0)
    animation   = go.Figure(data=frame_data)

    animation   = create_groundtrack_animation(animation, satellite, planet, update_rate)
    animation.update_geos(projection_type="equirectangular",
                    showland=True, 
                    coastlinewidth=2,
                    landcolor="rgb(52, 165, 111)",
                    showocean=True,
                    oceancolor="rgb(0, 204, 255)",
                    showcountries=True, countrycolor="darkgreen",
                    showlakes=True, lakecolor="rgb(0, 130, 255)",
                    showrivers=True, rivercolor="rgb(0, 130, 255)",
                    lonaxis=dict(dtick=20, showgrid=True, gridcolor="grey"),
                    lataxis=dict(dtick=20, showgrid=True, gridcolor="grey"),
                    resolution=50)
    return animation

def ecef(satellite, planet, update_rate):
    animation = groundtrack(satellite, planet, update_rate)
    animation.update_geos(projection_type="orthographic")
    return animation