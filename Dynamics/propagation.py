import numpy as np
import quaternion

def gravity(r, mu):
    '''
    Calculates the gravitational acceleration at a point
    :param r: position vector
    :param mu: gravitational parameter of the central body
    :return: gravitational acceleration vector
    '''
    return -mu*r/(np.linalg.norm(r)**3)

def J2_perturbation(r, mu, R, J2):
    '''
    Calculates the J2 perturbation acceleration at a point
    :param r: position vector
    :param mu: gravitational parameter of the central body
    :param R: radius of the central body
    :param J2: J2 coefficient of the central body
    :return: J2 perturbation acceleration vector
    '''
    r_mag   = np.linalg.norm(r)
    J2_pert = -3/2 * J2 * mu / (r_mag**2) * (R/r_mag)**2 * np.array([(1 - 5*(r[2]/r_mag)**2)*r[0]/r_mag, 
                                                                     (1 - 5*(r[2]/r_mag)**2)*r[1]/r_mag, 
                                                                     (3 - 5*(r[2]/r_mag)**2)*r[2]/r_mag])
    return J2_pert

def state_dot(t, state, mass, J, T_body, L_body, central_body):
    pos     = state[0:3]
    vel     = state[3:6]
    quat    = np.quaternion(state[6], state[7], state[8], state[9])
    omega   = state[10:13]

    R = central_body.radius
    mu = central_body.mu
    J2 = central_body.J2

    grav    = gravity(pos, mu)
    J2_pert = J2_perturbation(pos, mu, R, J2)

    T_body  = quaternion.rotate_vectors(quat, T_body)

    p_dot   = vel
    v_dot   = grav + J2_pert + T_body/mass
    q_dot   = -1/2 * np.quaternion(0, omega[0], omega[1], omega[2]) * quat
    alpha   = np.linalg.inv(J) @ (L_body - np.cross(omega,  J @ omega))

    x_dot   = np.hstack((p_dot, v_dot, q_dot.w, q_dot.x, q_dot.y, q_dot.z, alpha))
    return x_dot