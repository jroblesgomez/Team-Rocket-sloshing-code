# -*- coding: utf-8 -*-
"""
Author: Miriam García Medina
Date: 10th March 2019
email: migarme@etsid.upv.es

Sloshing Rocket Workshop Competition
"""

import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt


def fluid_column_height_variation(r1, r2, rho, h0, H, p0, n_dt):
    """
    Calculum of the variation of the height of the fluid column during time

    Parameters
    ----------
    r1   : Radius of the cylinder (m)
    r2   : Radius of the exit hole (m)
    rho  : Density of the fluid (kg/m3)
    h0   : Initial height of the fluid column (m)
    H    : Pressured cylinder height (m)
    p0   : Initial pressure of the cylinder (Pa)
    n_dt : Number of time intervals for the calculation.

    Output
    ------
    t, h : Arrays.
           Height of the fluid column in each time instant
    """

    patm = 1.013e5
    g = 9.81

    s1 = np.pi * r1 ** 2
    s2 = np.pi * r2 ** 2

    t = np.zeros(n_dt)
    h = np.linspace(0, h0, n_dt)

    c = 2 * p0 * (H - h0) / ((1 - s2 ** 2 / s1 ** 2) * rho)
    a = rho * g / (p0 * (H - h0))
    b = patm / (p0 * (H - h0))
    d = -s2/s1

    for i in range(n_dt):
        t[i] = integrate.quad(lambda h, a, b, c, d, H: 1 / (d * np.sqrt(c * (1 / (H - h) + a * h - b))),
                              h0, h[i], args=(a, b, c, d, H))[0]

    return np.flip(t), np.flip(h)


def empty_velocity(h, t, r1, r2):
    """
    Calculum of the cylinder empty velocity variation among time

    Parameters
    ----------
    h    : Variation of fluid column height array
    t    : Time array
    r1   : Radius of the cylinder (m)
    r2   : Radius of the exit hole (m)

    Output
    ------
    v1 : Array
         Upper surface fluid velocity during time
    v2 : Array
         Empty fluid velocity during time
    """

    s1 = np.pi * r1 ** 2
    s2 = np.pi * r2 ** 2

    v1 = np.zeros(len(t))

    for i in range(len(t) - 1):
        v1[i + 1] = (h[i + 1] - h[i]) / (t[i + 1] - t[i])

    v2 = (s1 / s2) * v1

    return v1, v2


def empty_mass(rho, r1, v1):
    """
    Calculum of the mass flow rate
    
    Parameters
    ----------
    rho : Density of the fluid (kg/m3)
    r1  : Radius of the cylinder (m)
    v1  : Upper surface fluid velocity array (m/s)
        
    Output
    ------
    m : Array
        Mass flow rate during time     
    """
    
    s1 = np.pi * (r1 ** 2)
    m = rho * s1 * (-v1)

    return m


def mass_test(m, t, r1, h0, rho):
    """
    Calculation of the remaining water volum in the tank
    
    Parameters
    ----------
    m   : Mass flow rate (kg/s)
    t   : Time array (s)
    r1  : Radius of the cylinder (m)
    h0  : Initial height of the fluid column (m)
    rho : Density of the fluid (kg/m3)
        
    Output
    ------
    difference : remaining water volum in the tank 
    """
    
    total_mass = 0
    for i in range(len(m) - 1):
        total_mass += ((m[i + 1] + m[i]) / 2) * (t[i + 1] - t[i])

    difference = np.pi * (r1 ** 2) * h0 * rho - total_mass

    return difference


def weight(m, t, m_rocket, r1, h0, rho, n_dt):
    """
    Calculum of weight during water outlet stage
    
    Parameters
    ----------
    m        : Mass flow rate (kg/s)
    t        : Time array (s)
    m_rocket : Fixed mass and sloshing mass (kg)
    r1       : Radius of the cylinder (m)
    h0       : Initial height of the fluid column (m)
    rho      : Density of the fluid (kg/m3)
    n_dt     : Number of time intervals for the calculation.
        
    Output
    ------
    w       : Array
              Weight 
    m_total : Array
              Mass in each time instant
    """
    
    g = 9.81
    w = np.zeros(n_dt)
    m_total = np.zeros(n_dt)

    m_water = np.zeros(n_dt)
    m_water[0] = rho * np.pi * (r1 ** 2) * h0

    for i in range(n_dt - 1):
        m_water[i + 1] = m_water[i] - m[i] * (t[i + 1] - t[i])

    for i in range(n_dt):
        w[i] = g * (m_rocket + m_water[i])
        m_total[i] = m_rocket + m_water[i]

    return w, m_total


def resistance(v_rocket):
    """
    Calculum of resistance during water outlet stage
    
    Parameters
    ----------
    v_rocket : Velocity of the rocket array (m/s)
        
    Output
    ------
    resistance depending on the speed of the rocket during water outlet stage
    """
    
    cd = 0.015
    rho_air = 1.225  # kg/m3
    s_wing = 0.555  # m2

    return 0.5 * cd * rho_air * s_wing * v_rocket ** 2


def thrust(n_dt, m, v2, w, m_total, t):
    """
    Calculum of the cinematic properties durig water outlet stage
    
    Parameters
    ----------
    n_dt    : Number of time intervals for the calculation.
    m       : Mass flow rate (kg/s)
    v2      : Empty fluid velocity array (m/s)
    w       : Weight array (N)
    m_total : Mass array (kg)
    t       : Time array (s)
        
    Output
    ------
    a_rocket : Array
               Acceleration of the rocket in each time instant
    v_rocket : Array
               Velocity of the rocket in each time instant
    h_rocket : Array
               Height of the rocket in each time instant
    """
    
    a_rocket = np.zeros(n_dt)
    v_rocket = np.zeros(n_dt)
    h_rocket = np.zeros(n_dt)

    for i in range(n_dt - 1):
        a_rocket[i + 1] = ((m[i] * (-v2[i])) - w[i] - resistance(v_rocket[i])) / m_total[i + 1]
        v_rocket[i + 1] = v_rocket[i] + a_rocket[i + 1] * (t[i + 1] - t[i])
        h_rocket[i + 1] = h_rocket[i] + v_rocket[i + 1] * (t[i + 1] - t[i])

    return a_rocket, v_rocket, h_rocket


def non_thrust(t_thrust_end, a_rocket_thrust_end, v_rocket_thrust_end, h_rocket_thrust_end, m_rocket, n_dt2):
    """
    Calculum of the cinematic properties when the water is over 
    
    Parameters
    ----------
    t_thrust_end        : Time final element array after thrust (s)
    a_rocket_thrust_end : Acceleration final element array after thrust (m/s2)
    v_rocket_thrust_end : Velocity final element array after thrust (m/s)
    h_rocket_thrust_end : Height final element array after thrust (m)      
    m_rocket            : Fixed mass and sloshing mass (kg)
    n_dt2               : Number of time intervals for the calculation.
        
    Output
    ------
    t        : Array
               Time 
    a_rocket : Array
               Acceleration of the rocket in each time instant
    v_rocket : Array
               Velocity of the rocket in each time instant
    h_rocket : Array
               Height of the rocket in each time instant 
    """
    
    s_wing = 80 * 20 * 1e-4  # m2
    g = 9.81

    a_rocket = np.zeros(n_dt2)
    v_rocket = np.zeros(n_dt2)
    h_rocket = np.zeros(n_dt2)

    a_rocket[0] = a_rocket_thrust_end
    v_rocket[0] = v_rocket_thrust_end
    h_rocket[0] = h_rocket_thrust_end

    t = np.linspace(t_thrust_end, t_thrust_end + 3, n_dt2)

    for i in range(n_dt2 - 1):
        a_rocket[i + 1] = (-g * m_rocket - resistance(v_rocket[i])) / m_rocket
        v_rocket[i + 1] = v_rocket[i] + a_rocket[i + 1] * (t[i + 1] - t[i])
        h_rocket[i + 1] = h_rocket[i] + v_rocket[i + 1] * (t[i + 1] - t[i])

        if v_rocket[i + 1] <= 0:
            break

    return t, a_rocket, v_rocket, h_rocket


##############################################################################
########################## PROBLEM EXECUTION #################################
##############################################################################

def rocket_water_optimality(p_h0, fig=False):
    H = 0.345 # m
    h0 = H * p_h0  # m

    r1 = 0.0375  # m
    r2 = 0.0125  # m

    patm = 1.013e5  # Pa
    p0 = 5 * patm  # Pa

    rho = 1000  # kg/m3

    m_fixed = 0.7  # kg
    m_sloshing = rho * np.pi * r1 ** 2 * h0  # kg
    m_rocket = m_sloshing + m_fixed  # kg

    n_dt = 50000
    n_dt2 = 50000

    (t_thrust, h) = fluid_column_height_variation(r1, r2, rho, h0, H, p0, n_dt)

    v1, v2 = empty_velocity(h, t_thrust, r1, r2)

    m = empty_mass(rho, r1, v1)
    difference = mass_test(m, t_thrust, r1, h0, rho)*1000

    w, m_total = weight(m, t_thrust, m_rocket, r1, h0, rho, n_dt)

    a_rocket_thrust, v_rocket_thrust, h_rocket_thrust = thrust(n_dt, m, v2, w, m_total, t_thrust)
    t_nonthrust, a_rocket_nonthrust, v_rocket_nonthrust, h_rocket_nonthrust = non_thrust(t_thrust[n_dt - 1],
                                                                                         a_rocket_thrust[n_dt - 1],
                                                                                         v_rocket_thrust[n_dt - 1],
                                                                                         h_rocket_thrust[n_dt - 1],
                                                                                         m_rocket, n_dt2)

    res_thrust = np.zeros(len(v_rocket_thrust))
    for i in range(len(v_rocket_thrust)):
        res_thrust[i] = resistance(v_rocket_thrust[i])

    h_total = np.append(h_rocket_thrust, h_rocket_nonthrust)
    limit = np.argmax(h_total)
    h_total = h_total[:limit + 1]
    t_total = np.append(t_thrust, t_nonthrust)[:limit + 1]
    a_total = np.append(a_rocket_thrust, a_rocket_nonthrust)[:limit + 1]
    v_total = np.append(v_rocket_thrust, v_rocket_nonthrust)[:limit + 1]

    if fig == True:
        f, axarr = plt.subplots(3, 1)
        axarr[0].plot(t_total[3:], h_total[3:])
        axarr[0].set_xlabel('Time (s)')
        axarr[0].set_ylabel('Height (m)')
        axarr[1].plot(t_total[3:], v_total[3:])
        axarr[1].set_xlabel('Time (s)')
        axarr[1].set_ylabel('Velocity (m/s)')
        axarr[2].plot(t_total[3:], a_total[3:])
        axarr[2].set_xlabel('Time (s)')
        axarr[2].set_ylabel('Acceleration (m/s^2)')
        f.suptitle('Rocket parameters from launch until maximum height')
        plt.savefig('results_figure.pdf')
        a=np.max(h_rocket_nonthrust)
        print('The maximum height is %f'%a)
        
    return np.max(h_rocket_nonthrust)


def optimum_h0(n, H):
    p_h0 = np.linspace(0.15, 0.9, n)
    h_rocket_fill = np.zeros(n)

    for i in range(n):
        h_rocket_fill[i] = rocket_water_optimality(p_h0[i])

    f, ax = plt.subplots()
    ax.plot(p_h0, h_rocket_fill)
    ax.set_title('Optimum water fill')
    ax.set_xlabel('h0 (%)')
    ax.set_ylabel('Rocket height (m)')
    plt.savefig('optimization_figure_h0.pdf')
    
    return p_h0, h_rocket_fill




optimum_h0(20, 0.345)

rocket_water_optimality(0.35, fig=True)
