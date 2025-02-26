import quaternion
import numpy as np
import utils.OEConvert as OEConvert

# def icrs2eci(icrsState, earth_state):
#     axial_tilt  = np.radians(23.44)
#     icrsState   = icrsState - earth_state
#     dcm         = np.array([[1, 0,                      0],
#                             [0, np.cos(axial_tilt),     np.sin(axial_tilt)],
#                             [0, -np.sin(axial_tilt),    np.cos(axial_tilt)]])
#     eciState        = np.zeros(icrsState.shape)
#     eciState[0:3]   = np.matmul(dcm, icrsState[0:3].T).T
#     eciState[3:6]   = np.matmul(dcm, icrsState[3:6].T).T
#     return eciState

# def eci2icrs(eciState, earth_state):
#     axial_tilt  = np.radians(23.44)
#     dcm         = np.array([[1, 0,                  0],
#                             [0, np.cos(axial_tilt), -np.sin(axial_tilt)],
#                             [0, np.sin(axial_tilt), np.cos(axial_tilt)]])
#     icrsState       = np.zeros(eciState.shape)
#     icrsState[0:3]  = np.matmul(dcm, eciState[0:3].T).T
#     icrsState[3:6]  = np.matmul(dcm, eciState[3:6].T).T
#     return icrsState + earth_state

def dir_inertial_to_radec(vector_inertial):
    dist    = np.linalg.norm(vector_inertial)
    dir     = vector_inertial/np.linalg.norm(vector_inertial)

    RA      = np.atan2(dir[1], dir[0])
    DEC     = np.arcsin(dir[2]/dist)
    return np.array([RA, DEC])


def pci2pcpf(pci_state, planet, days_since_j2000):
        q_inertial_to_pcpf, omega_pcpf = get_pci_to_pcpf(planet, days_since_j2000)
        pcpf_state      = np.zeros(6)
        pcpf_state[0:3] = quaternion.rotate_vectors(q_inertial_to_pcpf, pci_state[0:3])
        pcpf_state[3:6] = quaternion.rotate_vectors(q_inertial_to_pcpf, pci_state[3:6]) \
                            + np.cross(omega_pcpf, pcpf_state[0:3])
        return pcpf_state

def pcpf2pci(pcpf_state, planet, days_since_j2000):
    q_pcpf_to_inertial, omega_pcpf = get_pcpf_to_pci(planet, days_since_j2000)
    pci_state       = np.zeros(6)
    pci_state[0:3]  = quaternion.rotate_vectors(q_pcpf_to_inertial, pcpf_state[0:3])
    pci_state[3:6]  = quaternion.rotate_vectors(q_pcpf_to_inertial, pcpf_state[3:6]) \
                        + np.cross(omega_pcpf, pci_state[0:3])
    return pci_state

def get_pci_to_pcpf(planet, days_since_j2000):
    degrees_per_day     = 360 / planet.sidereal_day * (24*3600)
    rot_offset          = planet.rot_offset
    gamma               = np.radians(degrees_per_day*days_since_j2000 + rot_offset)
    q_inertial_to_pcpf  = quaternion.from_rotation_vector(-gamma*np.array([0, 0, 1]))
    omega_pcpf  = np.array([0, 0, degrees_per_day/(24*3600)])
    return q_inertial_to_pcpf, omega_pcpf

def get_pcpf_to_pci(planet, days_since_j2000):
    degrees_per_day     = 360 / planet.sidereal_day * (24*3600)
    rot_offset          = planet.rot_offset
    gamma               = np.radians(degrees_per_day*days_since_j2000 + rot_offset)
    q_pcpf_to_inertial  = quaternion.from_rotation_vector(gamma*np.array([0, 0, 1]))
    omega_pcpf  = np.array([0, 0, -degrees_per_day/(24*3600)])
    return q_pcpf_to_inertial, omega_pcpf

def get_hill_to_pci(state, mu):
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
    q_hill_to_inertial = quaternion.from_rotation_matrix(R_hill_to_inertial)

    a           = OEConvert.semimajor_axis(state, mu)
    n           =  np.sqrt(mu/a**3)
    omega_hill  = np.array([0, 0, n])
    return q_hill_to_inertial, omega_hill

def get_lvlh_to_pci(state, mu):
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
    q_lvlh_to_inertial = quaternion.from_rotation_matrix(R_lvlh_to_inertial)
    
    a           = OEConvert.semimajor_axis(state, mu)
    n           =  np.sqrt(mu/a**3)
    omega_lvlh  = np.array([0, n, 0])
    return q_lvlh_to_inertial, omega_lvlh

def pcpf2lla(pcpfState, planet):
    x = pcpfState[0]
    y = pcpfState[1]
    z = pcpfState[2]

    # WGS84 ellipsoid parameters
    a = planet.radius
    b = planet.polar_radius

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

def lla2pcpf(llaState, planet):
    # unpacking the input vector
    lat = np.radians(llaState[0])
    lon = np.radians(llaState[1])
    alt = llaState[2]

    # WGS84 ellipsoid parameters
    planet_semimajor = planet.radius
    planet_semiminor = planet.polar_radius

    # Flattening of the ellipsoid (WGS84)
    f = (planet_semimajor - planet_semiminor) / planet_semimajor

    # Eccentricity of the reference ellipsoid (WGS84)
    e = np.sqrt(f * (2 - f))

    # Distance between z axis and normal to ellipsoid
    N = planet_semimajor / np.sqrt(1 - e**2 * (np.sin(lat))**2)

    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (1 - e**2) + alt) * np.sin(lat)
    return np.array([x, y, z])