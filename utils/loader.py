import os
import yaml
import quaternion
import numpy as np

import Core.satellite as sat
import Core.simulator as sim
import Dynamics.body as body
import utils.OEConvert as OEConvert

def populate_sim(sim_file):
    with open(sim_file, 'r') as file:
        config      = yaml.safe_load(file)

    with open("Config/planets.yaml", 'r') as planet_file:
        planet_conf = yaml.safe_load(planet_file)

    sim_properties  = config["sim"]
    body_name       = sim_properties["central_body"]
    tf              = sim_properties["tf"]
    dt              = sim_properties["dt"]
    save_loc        = sim_properties["save_loc"]
    save_file       = sim_properties["save_file"]
    save_file       = os.path.join(save_loc, save_file)

    central_body    = body.Body(body_name, planet_conf[body_name])

    sats            = []
    satellites      = config["satellites"]

    for key in satellites.keys():
        satellite   = satellites[key]

        sat_config  = satellite["vehicle_conf"]
        sat_props   = yaml.safe_load(open(sat_config, 'r'))

        name        = sat_props["name"]
        mass        = sat_props["mass"]
        J           = np.diag(sat_props["inertia"])
        model       = sat_props["model"]
        axis_order  = sat_props["axis_order"]
        lights      = np.array([sat_props["nav_red"], 
                                sat_props["nav_green"]])

        if "cartesian" in satellite.keys():
            r       = np.array(satellite["cartesian"]["r"])
            v       = np.array(satellite["cartesian"]["v"])
            state   = np.hstack((r, v))
        elif "keplerian" in satellite.keys():
            kep = satellite["keplerian"]
            if "alt" in kep.keys():
                a   = kep["alt"] + central_body.radius
            elif "semimajor" in kep.keys():
                a   = kep["semimajor"]
            else:
                print("No altitude or semimajor axis specified")
                exit()
            e       = kep["ecc"]
            i       = kep["inc"]
            raan    = kep["raan"]
            argp    = kep["argp"]
            TA      = kep["TA"]

            state   = OEConvert.keplerian_to_cartesian([a, e, i, raan, argp, TA], central_body.mu)
        else:
            print("No initial state specified")
            exit()

        quat        = np.array(satellite["q_body_to_inertial"])
        omega_body  = np.array(satellite["omega_body"])
        state       = np.hstack((state, quat, omega_body))

        mass_prop   = (mass, J)
        visual_prop = (model, axis_order, lights)
        satellite   = sat.Satellite(name, state, mass_prop, visual_prop)
        sats.append(satellite)

    simulator = sim.Simulator(central_body, tf, dt, sats, save_file)
    return simulator