import numpy as np
import plotly.graph_objects as go

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

def eci(spacecraft, body):  
    planet = draw_planet(body)
    objs = [planet]
    for sc in spacecraft:
        state_hist = sc.state_history
        traj = go.Scatter3d(x=state_hist[:,0], y=state_hist[:,1], z=state_hist[:,2], mode="lines",
                            line=dict(width=4, color=np.arange(len(state_hist)), colorscale=sc.colorscale), name=sc.name)
        objs.append(traj)
    
    fig = go.Figure(data=objs)
    fig.update_layout(title="ECI Trajectory",scene = dict(xaxis_title='X [m]',
                                                          yaxis_title='Y [m]',
                                                          zaxis_title='Z [m]',
                                                            aspectmode="cube"),
                                                font=dict(family="Courier New, monospace",
                                                          size=18,color="RebeccaPurple"))
    return fig

def ecef(spacecraft): 
    fig = go.Figure();  
    for sc in spacecraft:
        state_lla_hist = sc.lla_hist
        fig.add_trace(go.Scattergeo(lat=state_lla_hist[:,0], lon=state_lla_hist[:,1], 
                                    mode="markers", marker=dict(size=4, color=np.arange(len(state_lla_hist)), 
                                                                colorscale=sc.colorscale), name=sc.name))
        fig.add_trace(go.Scattergeo(lat=[state_lla_hist[-1,0]], lon=[state_lla_hist[-1,1]],mode="markers", 
                                    marker=dict(size=15, symbol = "star-triangle-up"), name=sc.name + " End", 
                                    showlegend=False))
    fig.update_layout(title="ECEF Trajectory",
                      font=dict(family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"))
    fig.update_geos(projection_type="orthographic",
                    showland=True, 
                    landcolor="rgb(52, 165, 111)",
                    showocean=True,
                    oceancolor="rgb(0, 204, 255)",
                    showcountries=True, countrycolor="Black",
                    showlakes=True, lakecolor="rgb(0, 130, 255)",
                    showrivers=True, rivercolor="rgb(0, 130, 255)",
                    lonaxis=dict(dtick=10, showgrid=True),
                    lataxis=dict(dtick=10, showgrid=True))
    return fig

def ground_track(spacecraft): 
    fig = go.Figure();  
    for sc in spacecraft:
        state_lla_hist = sc.lla_hist
        fig.add_trace(go.Scattergeo(lat=state_lla_hist[:,0], lon=state_lla_hist[:,1], 
                                    mode="markers", marker=dict(size=4, color=np.arange(len(state_lla_hist)), 
                                                                colorscale=sc.colorscale), name=sc.name))
        fig.add_trace(go.Scattergeo(lat=[state_lla_hist[-1,0]], lon=[state_lla_hist[-1,1]],mode="markers", 
                                    marker=dict(size=15, symbol = "star-triangle-up"), name=sc.name + " End", 
                                    showlegend=False))
        
    fig.update_layout(title="Ground Track",
                      font=dict(family="Courier New, monospace",
                                size=18,
                                color="RebeccaPurple"))
    fig.update_geos(projection_type = "equirectangular",
                    showland=True, 
                    landcolor="rgb(52, 165, 111)",
                    showocean=True,
                    oceancolor="rgb(0, 204, 255)",
                    showcountries=True, countrycolor="Black",
                    showlakes=True, lakecolor="rgb(0, 130, 255)",
                    showrivers=True, rivercolor="rgb(0, 130, 255)",
                    lonaxis=dict(range=[-180, 180], dtick = 10, showgrid=True),
                    lataxis=dict(range=[-90, 90], dtick = 10, showgrid=True))
    return fig