import numpy as np
import utils.OEConvert as OEConvert

def icrs2eci(icrsState, earth_state):
    axial_tilt  = np.radians(23.44)
    icrsState   = icrsState - earth_state
    dcm         = np.array([[1, 0,                      0],
                            [0, np.cos(axial_tilt),     np.sin(axial_tilt)],
                            [0, -np.sin(axial_tilt),    np.cos(axial_tilt)]])
    eciState        = np.zeros(icrsState.shape)
    eciState[0:3]   = np.matmul(dcm, icrsState[0:3].T).T
    eciState[3:6]   = np.matmul(dcm, icrsState[3:6].T).T
    return eciState

def eci2icrs(eciState, earth_state):
    axial_tilt  = np.radians(23.44)
    dcm         = np.array([[1, 0,                  0],
                            [0, np.cos(axial_tilt), -np.sin(axial_tilt)],
                            [0, np.sin(axial_tilt), np.cos(axial_tilt)]])
    icrsState       = np.zeros(eciState.shape)
    icrsState[0:3]  = np.matmul(dcm, eciState[0:3].T).T
    icrsState[3:6]  = np.matmul(dcm, eciState[3:6].T).T
    return icrsState + earth_state

def eci2ecef(eciState, days_since_j2000):
    gamma   = np.radians(360.9856123035484*days_since_j2000 + 280.46)
    dcm     = np.array([[np.cos(gamma),     np.sin(gamma),  0],
                        [-np.sin(gamma),    np.cos(gamma),  0],
                        [0,                 0,              1]])
    ecefState       = np.zeros(eciState.shape)
    ecefState[0:3]  = np.matmul(dcm, eciState[0:3].T).T
    ecefState[3:6]  = np.matmul(dcm, eciState[3:6].T).T
    return ecefState

def ecef2eci(ecefState, days_since_j2000):
    gamma   = np.radians(360.9856123035484*days_since_j2000 + 280.46)
    dcm     = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma),  0],
                        [0,             0,              1]])
    eciState        = np.zeros(ecefState.shape)
    eciState[0:3]   = np.matmul(dcm, ecefState[0:3].T).T
    eciState[3:6]   = np.matmul(dcm, ecefState[3:6].T).T
    return eciState

def get_ecef_to_inertial(days_since_j2000):
    degrees_per_day     = 360.9856123035484
    gamma               = np.radians(degrees_per_day*days_since_j2000 + 280.46)
    R_ecef_to_inertial  = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                                    [np.sin(gamma), np.cos(gamma),  0],
                                    [0,             0,              1]])
    omega_ecef  = np.array([0, 0, degrees_per_day/(24*3600)])
    return R_ecef_to_inertial, omega_ecef
    

def get_hill_to_inertial(state, mu_earth):
    '''
    Calculates the rotation matrix from the Hill frame to the inertial frame
    :param state: current state vector [x, y, z, vx, vy, vz]
    :return: rotation matrix from the Hill frame to the inertial frame
    '''
    r = state[0:3]
    v = state[3:6]
    h = np.cross(r, v)

    r_normalized    = r/np.linalg.norm(r)
    h_normalized    = h/np.linalg.norm(h)

    # Rotation matrix from Hill to Inertial
    R_hill_to_inertial = np.array([r_normalized, np.cross(h_normalized, r_normalized), h_normalized]).T
    
    a           = OEConvert.semimajor_axis(state, mu_earth)
    n           =  np.sqrt(mu_earth/a**3)
    omega_hill  = np.array([0, 0, n])
    return R_hill_to_inertial, omega_hill

def get_lvlh_to_inertial(state, mu_earth):
    '''
    Calculates the rotation matrix from the LVLH frame to the inertial frame
    :param state: current state vector [x, y, z, vx, vy, vz]
    :return: rotation matrix from the LVLH frame to the inertial frame
    '''
    r = state[0:3]
    v = state[3:6]
    h = np.cross(r, v)

    r_normalized    = r/np.linalg.norm(r)
    h_normalized    = h/np.linalg.norm(h)

    # Rotation matrix from Hill to Inertial
    R_lvlh_to_inertial = np.array([np.cross(-h_normalized, -r_normalized), -h_normalized, -r_normalized]).T
    
    a           = OEConvert.semimajor_axis(state, mu_earth)
    n           =  np.sqrt(mu_earth/a**3)
    omega_lvlh  = np.array([0, n, 0])
    return R_lvlh_to_inertial, omega_lvlh

def ecef2lla(ecefState):
    x = ecefState[0]
    y = ecefState[1]
    z = ecefState[2]

    # WGS84 ellipsoid parameters
    a = 6378137.0
    b = 6356752.3142

    e = np.sqrt(1 - b**2 / a**2)

    epsilon = np.sqrt(a**2 / b**2 - 1)

    rho = np.sqrt(x**2 + y**2)

    p = np.abs(z)/(epsilon**2)

    s = rho**2 / (e**2 * epsilon**2)

    q = p**2 - b**2 + s

    u = p / np.sqrt(q)

    v = b**2 * u**2 / q

    P = 27*v*s/q

    Q = (np.sqrt(P + 1) + np.sqrt(P))**(2/3)

    t = (1 + Q + 1/Q) / 6

    c = np.sqrt(u**2 - 1 + 2*t)

    w = (c - u) / 2

    d = np.sign(z) * np.sqrt(q) * (w + (np.sqrt(t**2 + v) - u*w - t/2 - 1/4)**(1/2))

    N = a * np.sqrt(1 + epsilon**2 * d**2 / b**2)

    lat = np.arcsin((epsilon**2 + 1) * (d/N))
    lon = np.arctan2(y, x)
    alt = rho * np.cos(lat) + z * np.sin(lat) - a**2/N
    return np.array([np.degrees(lat), np.degrees(lon), alt])

def lla2ecef(llaState):
    # unpacking the input vector
    lat = np.radians(llaState[0])
    lon = np.radians(llaState[1])
    alt = llaState[2]

    # WGS84 ellipsoid parameters
    earth_semimajor = 6378137.0
    earth_semiminor = 6356752.3142

    # Flattening of the ellipsoid (WGS84)
    f = (earth_semimajor - earth_semiminor) / earth_semimajor

    # Eccentricity of the reference ellipsoid (WGS84)
    e = np.sqrt(f * (2 - f))

    # Distance between z axis and normal to ellipsoid
    N = earth_semimajor / np.sqrt(1 - e**2 * (np.sin(lat))**2)

    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (1 - e**2) + alt) * np.sin(lat)
    return np.array([x, y, z])