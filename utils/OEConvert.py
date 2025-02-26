import numpy as np
import utils.format as disp

def cartesian_to_keplerian(state, mu):
    a       = semimajor_axis(state, mu)
    e       = np.linalg.norm(eccentricity(state, mu))
    i       = inclination(state)
    lan    = LAN(state)
    omega   = argument_of_periapse(state, mu)
    f       = true_anomaly(state, mu)
    return np.array([a, e, i, lan, omega, f])

def keplerian_to_cartesian(kep, mu):
    disp.kep_elem("Input", kepDeg2Rad(kep))
    pos         = position(kep, mu)
    vel         = velocity(kep, mu)

    state_vec   = np.array([pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]])
    disp.state_vec("Converted", state_vec)

    return state_vec
    # input = kepDeg2Rad(kep)
    # check = cartesian_to_keplerian(state_vec, mu)
    # if np.allclose(input, check, rtol=1e-04, atol=1e-05):
    #     return state_vec
    # else:
    #     print("WARNING: Keplerian elements do not match")
    #     disp.kep_elem("Input", input)
    #     disp.kep_elem("Output", check)
    #     return state_vec
        # exit(1)

def semimajor_axis(state, mu):
    r = np.linalg.norm(state[0:3])
    v = np.linalg.norm(state[3:6])

    if r == 0:
        return 0
    a = 1/(2/r - v**2/mu)
    return a

def eccentricity(state, mu):
    r       = state[0:3]
    v       = state[3:6]
    r_mag   = np.sqrt(np.dot(r, r))
    v_mag   = np.sqrt(np.dot(v, v))

    if r_mag == 0:
        return 0
    e       = (v_mag**2/mu - 1/r_mag)*r - 1/mu*np.dot(r, v)*v
    return e

def inclination(state):
    r       = state[0:3]
    v       = state[3:6]
    h       = np.cross(r, v)
    h_mag   = np.sqrt(np.dot(h, h))

    if h_mag == 0:
        return 0
    incl    = np.arccos(np.dot(h/h_mag, np.array([0, 0, 1])))
    return incl

def node_vector(state):
    r = state[0:3]
    v = state[3:6]
    h = np.cross(r, v)
    n = np.cross(np.array([0, 0, 1]), h)
    return n

def LAN(state):
    node        = node_vector(state)
    node_mag    = np.sqrt(np.dot(node, node))

    if node_mag == 0:
        return 0
    omega       = np.arccos(np.dot(node, np.array([1, 0, 0])) / node_mag)

    if np.dot(node, np.array([0, 1, 0])) < 0:
        omega = 2*np.pi - omega
    return omega

def argument_of_periapse(state, mu):
    node        = node_vector(state)
    node_mag    = np.sqrt(np.dot(node, node))
    e           = eccentricity(state, mu)
    e_mag       = np.sqrt(np.dot(e, e))

    if node_mag == 0 or e_mag == 0:
        return 0
    omega = np.arccos(np.dot(node, e)/(node_mag*e_mag))

    if (np.dot(e, np.array([0, 0, 1])) < 0):
        omega = 2*np.pi - omega
    return omega

def true_anomaly(state, mu):
    r       = state[0:3]
    e       = eccentricity(state, mu)
    r_mag   = np.sqrt(np.dot(r, r))
    e_mag   = np.sqrt(np.dot(e, e))

    if r_mag == 0 or e_mag == 0:
        return 0
    if np.isclose(np.dot(e, r)/(e_mag*r_mag), 1):
        return 0
    
    f = np.arccos(np.dot(e, r)/(e_mag*r_mag))
    while f >= 2*np.pi:
        f -= 2*np.pi
    while f <= -2*np.pi:
        f += 2*np.pi
    return f

def angular_momentum_from_OE(a, e, mu):
    if e >=0 and e < 1:
        return np.sqrt(mu*a*(1-e**2))
    elif e < 0:
        return np.sqrt(mu*a*(e**2-1))
    
def sso_inclination(a, e, planet):
    mu          = planet.mu
    R     = planet.radius
    J2    = planet.J2
    year_length = planet.year_length

    n_bar = np.sqrt(mu/a**3)
    lan_rate_sso = 2*np.pi/(year_length * 24 * 3600)
    i_rad = np.arccos(lan_rate_sso / ((-3/2 * (R/a)**2 * J2 * n_bar / (1-e**2)**2)))
    i_deg = np.degrees(i_rad)
    return i_deg

def synch_orbit(planet):
    mu          = planet.mu
    r           = planet.radius
    day_length  = planet.sidereal_day
    a           = ((day_length / (2*np.pi))**2 * mu)**(1/3)
    return a

def position(kep, mu):
    a       = kep[0]
    e       = kep[1]
    i       = np.radians(kep[2])
    lan    = np.radians(kep[3])
    omega   = np.radians(kep[4])
    f       = np.radians(kep[5])

    r       = a*(1-e**2)/(1+e*np.cos(f))
    theta   = omega + f
    pos     = r * np.array([np.cos(theta)*np.cos(lan) - np.cos(i)*np.sin(lan)*np.sin(theta),
                            np.cos(theta)*np.sin(lan) + np.cos(i)*np.cos(lan)*np.sin(theta),
                            np.sin(i)*np.sin(theta)])
    return pos

def velocity(kep, mu):
    a       = kep[0]
    e       = kep[1]
    inc     = np.radians(kep[2])
    lan    = np.radians(kep[3])
    omega   = np.radians(kep[4])
    f       = np.radians(kep[5])

    theta   = omega + f

    h       = angular_momentum_from_OE(a, e, mu)
    vel_vec = mu/h*np.array([-(np.cos(lan)*(np.sin(theta) + e*np.sin(omega)) + np.sin(lan)*(np.cos(theta)+ e*np.cos(omega))*np.cos(inc)),
                             -(np.sin(lan)*(np.sin(theta) + e*np.sin(omega)) - np.cos(lan)*(np.cos(theta) + e*np.cos(omega))*np.cos(inc)),
                             (np.cos(theta) + e*np.cos(omega))*np.sin(inc)])
                         
    return vel_vec

def kepRad2Deg(kep):
    return np.array([kep[0], kep[1], np.degrees(kep[2]), np.degrees(kep[3]), np.degrees(kep[4]), np.degrees(kep[5])])

def kepDeg2Rad(kep):
    return np.array([kep[0], kep[1], np.radians(kep[2]), np.radians(kep[3]), np.radians(kep[4]), np.radians(kep[5])]) 