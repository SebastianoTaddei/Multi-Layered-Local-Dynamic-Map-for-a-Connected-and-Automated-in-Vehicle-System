"""--------------------------------------------------------------------------
    This file is meant to handle all the commonly used functions.
   --------------------------------------------------------------------------"""

''' IMPORTS '''
# numpy for math
import numpy as np

# scipy for complex math
import scipy.linalg as linalg
from scipy.stats import multivariate_normal
from scipy.optimize import linear_sum_assignment

# interval for mathematical intervals
from interval import interval, inf

# pyclothoid to use Bertolazzi's clothoid library
from pyclothoids import Clothoid

# pymap3d to convert from ECEF to ENU
import pymap3d as pm

# pandas for easy data conversion
import pandas as pd

''' GLOBAL VARIABLES '''
# Internal IDs for detections and predictions
def init_det_ID():
    global det_ID
    det_ID = 0

# Paths where only non motorised vehicles can go into
def init_paths():
    global paths
    paths = ['footway', 'bridleway', 'steps', 'corridor', 'path', 'cycleway']

# Detection entity classes
def init_det_classes():
    global motor_det
    global non_motor_det
    motor_det     = ['Unknown', 'Car', 'Truck', 'Cycle']
    non_motor_det = ['Unknown', 'Pedestrian', 'Bicycle']

# Connection entity classes
def init_conn_classes():
    global motor_conn
    global non_motor_conn
    motor_conn     = ['Unknown', 'Moped', 'Motorcycle', 'PassengerCar', 'Bus',
                      'LightTruck', 'HeavyTruck', 'Trailer', 'SpecialVehicles',
                      'Tram', 'RoadSideUnit']
    non_motor_conn = ['Unknown', 'Pedestrian', 'Cyclist', 'RoadSideUnit']

# Initialise the local coordinate point origin for the tangent approximation
# and the rotation matrix from ECEF to ENU
def init_lcpo():
    global lcpo
    global ecef_to_enu
    lcpo        = [None]*6
    ecef_to_enu = [None]*9

def set_lcpo(lat, lon, height, ecef_x, ecef_y, ecef_z):
    global lcpo
    global ecef_to_enu
    lcpo        = [lat, lon, height, ecef_x, ecef_y, ecef_z]
    ecef_to_enu = [-np.sin(np.radians(lon)),                          np.cos(np.radians(lon)),                         0,
                   -np.sin(np.radians(lat))*np.cos(np.radians(lon)), -np.sin(np.radians(lat))*np.sin(np.radians(lon)), np.cos(np.radians(lat)),
                    np.cos(np.radians(lat))*np.cos(np.radians(lon)),  np.cos(np.radians(lat))*np.sin(np.radians(lon)), np.sin(np.radians(lat))
                   ] # flattened to be used in neo4j

# Initialise the match counters to measure the matching performance
def init_match_counters():
    global total_matches
    global correct_matches
    global maybe_matches
    global wrong_matches
    global unmatched_matches
    total_matches     = 0
    correct_matches   = 0
    maybe_matches     = 0
    wrong_matches     = 0
    unmatched_matches = 0

''' FUNCTIONS '''
# Format labels: ":Label1:Label2"
def format_labels(labels):
    return ':'.join(labels)

# Format properties: "{key1:value1, key2:value2}"
def format_properties(properties, id):
    return ','.join(('{0}:${1}.{0}'.format(i, id) for i in properties.keys()))

''' MATCHING ALGORITHM '''
# Predict next state
def predict_state(m, P, dt):
    # Initial data obtained heuristically from the data
    var_1, var_2 = 34.182878164359515, 2.0309133553083054

    # Containers
    m_out = np.zeros(m.shape)
    P_out = np.zeros(P.shape)

    # Predict for each state following an augmented quasi-constant turn model
    for i in range(m.shape[0]):
        px, py, v, psi, a, omega = m[i]

        F = np.array([[1, 0, np.cos(psi)*dt[i], -v*np.sin(psi)*dt[i], 0,     0],
                      [0, 1, np.sin(psi)*dt[i],  v*np.cos(psi)*dt[i], 0,     0],
                      [0, 0, 1,                  0,                   dt[i], 0],
                      [0, 0, 0,                  1,                   0,     dt[i]],
                      [0, 0, 0,                  0,                   1,     0],
                      [0, 0, 0,                  0,                   0,     1]])

        Q  = np.array([[0, 0, 0, 0, 0,           0],
                       [0, 0, 0, 0, 0,           0],
                       [0, 0, 0, 0, 0,           0],
                       [0, 0, 0, 0, 0,           0],
                       [0, 0, 0, 0, var_1*dt[i], 0],
                       [0, 0, 0, 0, 0,           var_2*dt[i]]])

        m_out[i] = m[i] + np.array([v*np.cos(psi)*dt[i],
                                    v*np.sin(psi)*dt[i],
                                    a*dt[i],
                                    omega*dt[i],
                                    0,
                                    0])
        P_out[i] = F @ P[i] @ F.T + Q

    return (m_out, P_out)

# Match detections
def match_detections(m, P, m_r, P_r):
    # Dynamic model
    dim_x = 6

    # Measurement model
    dim_y = 4

    # Kalman Filter
    def update(m, P, y, R):
        G = np.concatenate((np.eye(dim_y),
                            np.zeros((dim_y, dim_x - dim_y))),
                           axis=1)
        S = G @ P @ G.T + R
        K = P @ G.T @ linalg.inv(S)
        m = m + K @ (y - G @ m)
        P = P - K @ S @ K.T

        return (m, P)

    # Containers
    m_f  = np.zeros((np.size(m, axis=0), np.size(m_r, axis=0), dim_x))
    P_f  = np.zeros((np.size(m, axis=0), np.size(m_r, axis=0), dim_x, dim_x))
    cost = np.zeros((np.size(m, axis=0), np.size(m_r, axis=0)))

    # Match the measurements
    for i in range(np.size(m, axis=0)):
        for j in range(np.size(m_r, axis=0)):
            (m_f[i, j], P_f[i, j]) = update(m_r[j], P_r[j], m[i][0:4], P[i][0:4, 0:4])

            not_zero   = 1e-300 # avoid roundoff to zero
            cost[i, j] = (
                -np.log(multivariate_normal.pdf(m[i][0:4], m_f[i, j][0:4], P[i][0:4, 0:4]) + not_zero)
                -np.log(multivariate_normal.pdf(m_r[j][0:4], m_f[i, j][0:4], P_r[j][0:4, 0:4]) + not_zero)
            )

    # Solve the linear assignment problem
    (row_ind, col_ind) = linear_sum_assignment(cost)

    # Container
    idxs = []

    # Matches with positive cost are considered bad matches
    for i, cost_i in enumerate(cost[row_ind, col_ind]):
        if cost_i > 0:
            idxs.append(i)

    # Delete the bad matches
    match_row = np.delete(row_ind, idxs)
    match_col = np.delete(col_ind, idxs)

    # Get all non matched detections
    no_match_row = np.delete(np.arange(np.size(m, axis=0)), match_row)
    no_match_col = np.delete(np.arange(np.size(m_r, axis=0)), match_col)

    # Create the output including all matched and non matched detections (if
    # any)
    m_out = np.concatenate((m_f[match_row, match_col], m[no_match_row], m_r[no_match_col]))
    P_out = np.concatenate((P_f[match_row, match_col], P[no_match_row], P_r[no_match_col]))

    return (m_out, P_out, match_row, match_col, no_match_row, no_match_col)

# Match detections univocally
# One possible cause of failure is when the speed dramatically changes between
# timesteps.
def match(data, predicted_data):
    # Get global variables
    global det_ID
    global total_matches
    global correct_matches
    global maybe_matches
    global wrong_matches
    global unmatched_matches

    # Pre-fill data
    out = []
    if not data.empty:
        rest = [data[data['refID'] == its_id].to_dict(orient='list')
                for its_id in data['refID'].unique()]
    else:
        rest = []

    # Fill the missing fields of the incoming data
    for rest_i in rest:
        rows                 = len(rest_i['Lat'])
        rest_i['ID']         = [det_ID + i for i in range(rows)]
        det_ID              += rows
        rest_i['pred_steps'] = np.zeros(len(rest_i['refID']))
        rest_i['enu_x']          = [0]*rows
        rest_i['enu_y']          = [0]*rows
        rest_i['enu_z']          = [0]*rows
        rest_i['a']          = np.zeros(rows)
        rest_i['omega']      = np.zeros(rows)
        rest_i['P']          = np.zeros((rows, 6, 6))

        for i in range(rows):
            (rest_i['enu_x'][i],
             rest_i['enu_y'][i],
             rest_i['enu_z'][i]) = pm.geodetic2enu(rest_i['Lat'][i],
                                               rest_i['Lon'][i],
                                               0,
                                               lcpo[0], lcpo[1],
                                               lcpo[2])

            M   = np.diag([(rest_i['Smjr_ax_ellipse'][i])**2/5.991,
                           (rest_i['Smnr_ax_ellipse'][i] + 1e-4)**2/5.991])
            v12 = np.tan(np.radians(rest_i['Smjr_ellipse_orient_n'][i]))
            S   = np.array([[1, -v12],
                            [v12, 1]])
            P   = S @ M @ linalg.inv(S)

            rest_i['P'][i] = np.array([[P[0, 0], P[0, 1], 0,       0,                  0, 0],
                                       [P[1, 0], P[1, 1], 0,       0,                  0, 0],
                                       [0,       0,       0.2**2,  0,                  0, 0],
                                       [0,       0,       0,       np.radians(1.0)**2, 0, 0],
                                       [0,       0,       0,       0,                  0, 0],
                                       [0,       0,       0,       0,                  0, 0]])
            # Velocity and CourseOverGround heruistically determined from the data

        rest_i['m'] = np.array([rest_i['enu_x'],
                                rest_i['enu_y'],
                                np.asarray(rest_i['SpeedKMH'])/3.6,
                                np.radians(90 - np.asarray(rest_i['CourseOverGround'])),
                                rest_i['a'],
                                rest_i['omega']]).T

    # Check if there is old data to be matched
    if predicted_data['class']:
        predicted_data['pred_steps'] = np.asarray(predicted_data['pred_steps'])
        predicted_data['m']          = np.asarray(predicted_data['m']).reshape((len(predicted_data['class']), 6))
        predicted_data['P']          = np.asarray(predicted_data['P']).reshape((len(predicted_data['class']), 6, 6))

        rest.append(predicted_data)

    # If there are detections from the ego vehicle give it to out
    for i, rest_i in enumerate(rest):
        if 11112000 in rest_i['refID']:
            out = rest_i
            rest.pop(i)

    # Predict all detections to the current timestep
    for rest_i in rest:
        for i, delay in enumerate(rest_i['delay']):
            if delay > 0:
                (rest_i['m'],
                 rest_i['P']) = predict_state(rest_i['m'], rest_i['P'],
                                              rest_i['delay'])

                rest_i['pred_steps'][i] = rest_i['pred_steps'][i] + 1

    # If there aren't detections from the ego vehicle give the first ITS to out
    if len(out) == 0 and len(rest) != 0:
        out = rest[0]
        rest.pop(0)

    # For each other ITS match detections
    for rest_i in rest:
        (m_out, P_out,
         match_row,
         match_col,
         no_match_row,
         no_match_col) = match_detections(out['m'], out['P'], rest_i['m'],
                                          rest_i['P'])

        # Update match counters
        total_matches += len(out['handle'])

        for out_handle, rest_i_handle in zip(np.asarray(out['handle'])[match_row],
                                             np.asarray(rest_i['handle'])[match_col]):
            if out_handle == rest_i_handle:
                correct_matches += 1
            elif out_handle != rest_i_handle and out_handle in rest_i['handle']:
                wrong_matches += 1
            else:
                maybe_matches += 1

        for out_handle in np.asarray(out['handle'])[no_match_row]:
            if out_handle in rest_i['handle']:
                wrong_matches += 1
            else:
                unmatched_matches += 1

        # Update out fields
        for key in out.keys():
            # Keep the ID of the old data for persistency reasons
            if key == 'ID':
                matched = np.asarray(rest_i[key])[match_col]
            else:
                matched = np.asarray(out[key])[match_row]

            out[key] = np.concatenate([matched,
                                       np.asarray(out[key])[no_match_row],
                                       np.asarray(rest_i[key])[no_match_col]])

        out['m'] = m_out
        out['P'] = P_out

    # Create the detection data
    for i in range(len(out['refID'])):
        (out['Lat'][i],
         out['Lon'][i], _) = pm.enu2geodetic(out['m'][i][0],
                                             out['m'][i][1],
                                             out['enu_z'][i],
                                             *lcpo[0:3])
        (out['ecef_x'][i],
         out['ecef_y'][i],
         out['ecef_z'][i]) = pm.enu2ecef(out['m'][i][0],
                                         out['m'][i][1],
                                         out['enu_z'][i],
                                         *lcpo[0:3])

        out['enu_x'][i]            = out['m'][i][0]
        out['enu_y'][i]            = out['m'][i][1]
        out['SpeedKMH'][i]         = out['m'][i][2]*3.6
        out['CourseOverGround'][i] = 90 - np.degrees(out['m'][i][3])
        out['a'][i]                = out['m'][i][4]
        out['omega'][i]            = out['m'][i][5]

    detection      = {key: out[key] for key in list(out)[:-2]}
    detection['m'] = [list(out['m'][i]) for i in range(len(out['m']))]
    detection['P'] = [list(out['P'][i].flatten()) for i in range(len(out['P']))]
    detection      = pd.DataFrame.from_dict(detection).to_dict(orient='records')

    # Predict each detection to their next state (0.1 s)
    (out['m'], out['P']) = predict_state(out['m'], out['P'], [0.1]*len(out['m']))
    out['pred_steps']    = np.asarray(out['pred_steps']) + 1

    # Create the prediction data
    for i in range(len(out['refID'])):
        (out['Lat'][i],
         out['Lon'][i], _) = pm.enu2geodetic(out['m'][i][0],
                                             out['m'][i][1],
                                             out['enu_z'][i],
                                             *lcpo[0:3])
        (out['ecef_x'][i],
         out['ecef_y'][i],
         out['ecef_z'][i]) = pm.enu2ecef(out['m'][i][0],
                                         out['m'][i][1],
                                         out['enu_z'][i],
                                         *lcpo[0:3])

        out['enu_x'][i]                = out['m'][i][0]
        out['enu_y'][i]                = out['m'][i][1]
        out['SpeedKMH'][i]         = out['m'][i][2]*3.6
        out['CourseOverGround'][i] = 90 - np.degrees(out['m'][i][3])
        out['a'][i]                = out['m'][i][4]
        out['omega'][i]            = out['m'][i][5]

    prediction      = {key: out[key] for key in list(out)[:-2]}
    prediction['m'] = [list(out['m'][i]) for i in range(len(out['m']))]
    prediction['P'] = [list(out['P'][i].flatten()) for i in range(len(out['m']))]
    prediction      = pd.DataFrame.from_dict(prediction).to_dict(orient='records')

    return (detection, prediction)

''' MOTION PRIMITIVES AND ACTION SELECTION '''
# Adapted from slide 9 of Rosati.

# We use the closed form solution of the longitudinal kinematic model.
# More complex models can be used, but need to be chosen together with
# Biral.

# OCP coefficients, with t0 = s0 = 0
def ocp_coeffs(v0, a0, sf, vf, af, tf):
    c1 = v0
    c2 = a0
    c3 = (3*af - 9*a0)/tf + 60*sf/tf**3 - 12*(2*vf + 3*v0)/tf**2
    c4 = (36*a0 - 24*af)/tf**2 - 360*sf/tf**4 + 24*(7*vf + 8*v0)/tf**3
    c5 = 60*(af - a0)/tf**3 + 720*sf/tf**5 - 360*(vf + v0)/tf**4

    tot_cost = (c3**2*tf + c3*c4*tf**2 + 1/3*c3*c5*tf**3 + 1/3*c4**2*tf**3
                + 1/4*c4*c5*tf**4 + 4/5*c5**2*tf**5)

    return (np.array([c1, c2, c3, c4, c5]), tot_cost)

# OCP closed form solution
def ocp_sol(c, tf):
    # Create a discrete time vector
    t = np.linspace(0, tf, int(tf/1e-1) + 1)

    # Compute solutions over the time horizon
    t_2, t_3, t_4, t_5 = t**2, t**3, t**4, t**5
    s = c[0]*t + 1/2*c[1]*t_2 + 1/6*c[2]*t_3 + 1/24*c[3]*t_4 + 1/120*c[4]*t_5
    v = c[0] + c[1]*t + 1/2*c[2]*t_2 + 1/6*c[3]*t_3 + 1/24*c[4]*t_4
    a = c[1] + c[2]*t + 1/2*c[3]*t_2 + 1/6*c[4]*t_3
    j = c[2] + c[3]*t + 1/2*c[4]*t_2

    return np.stack([s, v, a, j], axis=1)

''' PRIMITIVES '''
# Stop primitive
def stop_primitive(v0, a0, sf):
    # Check if a solution exists at all
    if v0 <= 0 or sf == 0:
        return None

    # Since it is a stop motion vf = af = 0
    vf, af = 0, 0

    # Check if sf is too much
    if 4*v0**2 + 5*a0*sf < 0:
        sf = -4*v0**2/(5*a0)
        tf = 10*sf/(2*v0)
    else:
        tf = 10*sf/(2*v0 + np.sqrt(4*v0**2 + 5*a0*sf))

    # Compute the coefficients and the total cost
    (c, tot_cost) = ocp_coeffs(v0, a0, sf, vf, af, tf)

    # Return:
    # - jerk range;
    # - coefficients;
    # - total cost;
    # - final time;
    # - type.
    return {'jerk_range':interval([-inf, c[2]]), 'coeffs':c,
            'cost':tot_cost, 'tf':tf, 'type':'stop'}

# Stop primitive with j0 = 0
def stop_primitive_j0(v0, a0):
    # Check if a solutin exists at all
    if v0 > 0 and a0 < 0:
        # Optimal time to reach sf
        tf = -2*v0/a0

        # Final positition given j0 = 0
        sf = 1/60*tf*(9*a0*tf + 36*v0)

        # Final conditions given this is a stop primitive
        vf, af = 0, 0

        # Compute the coefficients and the total cost
        (c, tot_cost) = ocp_coeffs(v0, a0, sf, vf, af, tf)

        # Return:
        # - coefficients;
        # - total cost;
        # - final time;
        # - type.
        return {'coeffs':c, 'cost':tot_cost, 'tf':tf, 'type':'stop_j0'}
    else:
        return None

# Pass primitive
def pass_primitive(v0, a0, sf, vmin, vmax, tmin, tmax):
    # Compute the optimal time to reach vf
    def t_vf(sf, v0, vf, a0):
        return 30*sf/(7*v0 + 8*vf + np.sqrt(60*a0*sf + (7*v0 + 8*vf)**2))

    # Compute the optimal velocity
    def v_opt(tf, sf, v0, a0):
        return 15/8*sf/tf - a0*tf/8 - 7*v0/8

    # Since it is a pass motion af = 0
    af = 0

    # Check if there is a minimum velocity
    if a0 >= 0:
        t_vmin = t_vf(sf, v0, vmin, a0)
        t_vmax = t_vf(sf, v0, vmax, a0)
    else:
        t_star = np.sqrt(15*sf/(-a0))
        v_star = 1/4*(np.sqrt(-15*a0*sf) - 7*v0/2)

        if v_star < vmin < vmax:
            t_vmin = t_vf(sf, v0, vmin, a0)
            t_vmax = t_vf(sf, v0, vmax, a0)
        elif vmin < v_star < vmax:
            t_vmin = t_star
            t_vmax = t_vf(sf, v0, vmax, a0)
        else:
            t_vmin = 0
            t_vmax = 0

    t_12 = interval([tmin, tmax]) & interval([t_vmax, t_vmin])

    if len(t_12) != 0:
        t1, t2 = t_12[0].inf, t_12[0].sup
    else:
        t1, t2 = 0, 0

    if 0 < t1 <= t2:
        v_min = v_opt(t2, sf, v0, a0)
        v_max = v_opt(t1, sf, v0, a0)

        # Compute the two sets of coefficients
        (c1, tot_cost1) = ocp_coeffs(v0, a0, sf, v_max, af, t1)
        (c2, tot_cost2) = ocp_coeffs(v0, a0, sf, v_min, af, t2)

        # Return:
        # - jerk range;
        # - coefficients;
        # - total costs;
        # - final times;
        # - type.
        return [{'jerk_range':interval([c1[2], c2[2]]),
                 'coeffs':c1,
                 'cost':tot_cost1,
                 'tf':t1,
                 'type':'pass'},
                {'jerk_range':interval([c1[2], c2[2]]),
                 'coeffs':c2,
                 'cost':tot_cost2,
                 'tf':t2,
                 'type':'pass'}]
    else:
        return None

# Pass primitive with j0 = 0
def pass_primitive_j0(v0, a0, sf, vmin, vmax):
    # Optimal time to reach vf
    # Check for division by zero
    if (5*v0 - np.sqrt(5)*np.sqrt(8*a0*sf + 5*v0**2)) == 0:
        tf_12 = [1e6, # high number to avoid using this solution
                 10*sf/(5*v0 + np.sqrt(5)*np.sqrt(8*a0*sf + 5*v0**2))]
    elif (5*v0 + np.sqrt(5)*np.sqrt(8*a0*sf + 5*v0**2)) == 0:
        tf_12 = [10*sf/(5*v0 - np.sqrt(5)*np.sqrt(8*a0*sf + 5*v0**2)),
                 1e6] # high number to avoid using this solution
    else:
        tf_12 = [10*sf/(5*v0 - np.sqrt(5)*np.sqrt(8*a0*sf + 5*v0**2)),
                 10*sf/(5*v0 + np.sqrt(5)*np.sqrt(8*a0*sf + 5*v0**2))]

    # Optimal final velocity given j0 = 0
    vf_1 = 1/8*(20*sf/tf_12[0] - tf_12[0]*3*a0 - 12*v0)

    # Final acceleration given this is a pass primitive
    af = 0

    # Check if a solution exists
    if vmin < vf_1 < vmax:
        # Compute the coefficients and the total cost
        (c, tot_cost) = ocp_coeffs(v0, a0, sf, vf_1, af, tf_12[0])

        # Return:
        # - coefficients;
        # - total cost;
        # - final time;
        # - type.
        return {'coeffs':c, 'cost':tot_cost, 'tf':tf_12[0], 'type':'pass_j0'}
    else:
        # Optimal final velocity given j0 = 0
        vf_2 = 1/8*(20*sf/tf_12[1] - tf_12[1]*3*a0 - 12*v0)

        # Check if a solution exists
        if vmin < vf_2 < vmax:
            # Compute the coefficients and the total cost
            (c, tot_cost) = ocp_coeffs(v0, a0, sf, vf_2, af, tf_12[1])

            # Return:
            # - coefficients;
            # - total cost;
            # - final time;
            # - type.
            return {'coeffs':c, 'cost':tot_cost, 'tf':tf_12[1],
                    'type':'pass_j0'}
        else:
            return None

''' ACTION PREDICTION ALGORITHM '''

# Compute the most likely action of a detection
def predict_action(possible_actions):
    # Container for the output
    likely_actions   = []
    likely_clothoids = []

    # For each detection
    for (id, ecef_x, ecef_y, ecef_z, v0, a0, psi0, w0_id,
         n1_id, n1_ecef_x, n1_ecef_y, n1_ecef_z,
         n2_ids, n2_ecef_xs, n2_ecef_ys, n2_ecef_zs,
         w1_ids, speed_limits) in possible_actions:
        # Convert ECEF to ENU
        enu_x, enu_y, enu_z          = pm.ecef2enu(ecef_x, ecef_y, ecef_z,
                                                   *lcpo[0:3])
        n1_enu_x, n1_enu_y, n1_enu_z = pm.ecef2enu(n1_ecef_x, n1_ecef_y, n1_ecef_z,
                                                   *lcpo[0:3])

        # Container for the minimum jerks
        min_j = np.empty(0)

        # Container for the maximum lateral accelerations
        max_lat_a = np.empty(0)

        # Container for the clothoid to plot
        clothoid_c = []

        # Initialise the motion primitive and clothoid
        mp       = None
        clothoid = None

        # For each possible action
        for (n2_id, n2_ecef_x, n2_ecef_y, n2_ecef_z,
             w1_id, speed_limit) in zip(n2_ids, n2_ecef_xs, n2_ecef_ys, n2_ecef_zs,
                                        w1_ids, speed_limits):
            # Convert from ECEF to ENU
            n2_enu_x, n2_enu_y, n2_enu_z = pm.ecef2enu(n2_ecef_x, n2_ecef_y, n2_ecef_z,
                                                       *lcpo[0:3])

            clothoid = Clothoid.G1Hermite(enu_x, enu_y, np.radians(90 - psi0),
                                          n2_enu_x, n2_enu_y,
                                          np.arctan2(n2_enu_y - n1_enu_y,
                                                     n2_enu_x - n1_enu_x))

            # Longitudinal distance
            sf = clothoid.length

            # Compute velocity range
            if a0 > 0:
                # If it is accelerating it should go straight
                vmin, vmax = v0 - v0*0.05, speed_limit
            elif a0 < 0:
                # If it is decelerating it should turn or stop
                vmin, vmax = v0*0.05, v0 + v0*0.05
            elif a0 == 0:
                # If it isn't accelerating or decelerating it could do anything
                vmin, vmax = v0*0.05, speed_limit

            # Compute the time range
            tmin, tmax = sf/vmax, sf/vmin

            # Compute the stop primitive
            stop_mp = stop_primitive(v0, a0, sf)

            # Compute the pass primitive
            pass_mp = pass_primitive(v0, a0, sf, vmin, vmax, tmin, tmax)

            # Get the most likely motion primitive
            if stop_mp and pass_mp:
                if 0 in pass_mp[0]['jerk_range']:
                    min_j = np.append(min_j, 0)
                    mp    = pass_primitive_j0(v0, a0, sf, vmin, vmax)
                elif 0 in stop_mp['jerk_range']:
                    min_j = np.append(min_j, 0)
                    mp    = stop_primitive_j0(v0, a0)
                elif np.min(np.abs(stop_mp['jerk_range'])) < np.min(np.abs(pass_mp[0]['jerk_range'])):
                    min_j = np.append(min_j, np.min(np.abs(stop_mp['jerk_range'])))
                    mp    = stop_mp
                else:
                    min_j = np.append(min_j,
                                      np.min([np.abs(pass_mp[0]['coeffs'][3]),
                                              np.abs(pass_mp[1]['coeffs'][3])]))
                    mp    = pass_mp[np.argmin([np.abs(pass_mp[0]['coeffs'][3]),
                                               np.abs(pass_mp[1]['coeffs'][3])])]
            elif stop_mp:
                if 0 in stop_mp['jerk_range']:
                    min_j = np.append(min_j, 0)
                    mp    = stop_primitive_j0(v0, a0)
                else:
                    min_j = np.append(min_j, np.min(np.abs(stop_mp['jerk_range'])))
                    mp    = stop_mp
            elif pass_mp:
                if 0 in pass_mp[0]['jerk_range']:
                    min_j = np.append(min_j, 0)
                    mp    = pass_primitive_j0(v0, a0, sf, vmin, vmax)
                else:
                    min_j = np.append(min_j,
                                      np.min([np.abs(pass_mp[0]['coeffs'][3]),
                                              np.abs(pass_mp[1]['coeffs'][3])]))
                    mp    = pass_mp[np.argmin([np.abs(pass_mp[0]['coeffs'][3]),
                                               np.abs(pass_mp[1]['coeffs'][3])])]
            else:
                min_j = np.append(min_j, 1e10)

            # Compute the lateral jerk
            if clothoid and mp:
                # Evaluation points
                eval_pt = int(mp['tf']/1e-1) + 1

                # Curvilinear variable
                c_s = np.linspace(0, clothoid.length, eval_pt)

                # Curvature
                c_k = clothoid.dk*c_s + clothoid.KappaStart

                # Tangential velocity
                c_v = ocp_sol(mp['coeffs'], mp['tf'])[:, 1]

                # Lateral acceleration
                c_a = np.power(c_v, 2)*c_k

                # Get the maximum lateral acceleration
                max_lat_a = np.append(max_lat_a, np.max(np.abs(c_a)))

                # Append the clothoid
                clothoid_c.append([clothoid, mp['type']])
            else:
                max_lat_a = np.append(max_lat_a, 1e10)
                clothoid_c.append(None)

        if min_j.size != 0 and max_lat_a.size != 0:
            # Select the action that minimises both the initial longitudinal
            # jerk and the maximum lateral acceleration
            action_index = np.argmin(min_j + max_lat_a)

            if clothoid_c[action_index] != None:
                likely_actions.append([id, v0, a0, psi0, w0_id,
                                       n1_id,
                                       n2_ids[action_index],
                                       w1_ids[action_index],
                                       [ecef_x, ecef_y, ecef_z]])
                likely_clothoids.append(clothoid_c[action_index])

    return likely_actions, likely_clothoids

''' COLLISION AVOIDANCE ALGORITHM '''

# Compute an accurate estimate of the longitudinal distance using clothoids
def compute_sf(s0, psi0, nodes):
    # Convert from ECEF to ENU the entity position
    enu_x, enu_y, enu_z = pm.ecef2enu(s0[0], s0[1], s0[2],
                                      *lcpo[0:3])

    # Compute the first clothoid for the entity
    n_enu_x1, n_enu_y1, n_enu_z1 = pm.ecef2enu(nodes[0][0], nodes[0][1],
                                               nodes[0][2], *lcpo[0:3])

    # Assume to have the angle as the one between the nodes, since we are only
    # interested in a rough distance estimation it should be alright
    clothoid = Clothoid.G1Hermite(enu_x, enu_y, np.radians(90 - psi0),
                                  n_enu_x1, n_enu_y1,
                                  np.arctan2(n_enu_y1 - enu_y,
                                             n_enu_x1 - enu_x))

    # Sample points
    traj = np.asarray(clothoid.SampleXY(100)).T

    # Initialise the final distance
    sf = clothoid.length

    # For each node in the entity's path
    for node in nodes[1:]:
        # Convert from ECEF to ENU the node
        n_enu_x2, n_enu_y2, n_enu_z2 = pm.ecef2enu(node[0], node[1],
                                                   node[2], *lcpo[0:3])

        # Compute the clothoid
        clothoid = Clothoid.G1Hermite(n_enu_x1, n_enu_y1,
                                      clothoid.ThetaEnd,
                                      n_enu_x2, n_enu_y2,
                                      np.arctan2(n_enu_y2 - n_enu_y1,
                                                 n_enu_x2 - n_enu_x1))

        # Sample points
        traj = np.append(traj, np.asarray(clothoid.SampleXY(100)).T, axis=0)

        # Update the length
        sf += clothoid.length

        # Update the points
        n_enu_x1, n_enu_y1, n_enu_z1 = n_enu_x2, n_enu_y2, n_enu_z2

    # Sample points
    traj = np.append(traj, np.asarray(clothoid.SampleXY(100)).T, axis=0)

    # Update the length
    sf += clothoid.length

    # Return the final distance
    return sf, traj

# Compute the conditions for a collision
def predict_collision(possible_collisions):
    # Container for the initial conditions for the motion primitives of the ego
    # vehicle
    ego_init = []

    # Containers for the collision trajectories
    coll_trajs = []

    # For each possible collision
    for (a_s0, a_v0, a_a0, a_psi0, a_nodes, a_speed_limit, s0, v0, a0, psi0,
         nodes, speed_limit) in possible_collisions:
        # Compute the final longitudinal distances
        a_sf, a_traj = compute_sf(a_s0, a_psi0, a_nodes)
        sf, traj     = compute_sf(s0, psi0, nodes)

        # Initialise the collision variable
        coll_mp = None

        # Compute velocity range of the detection
        if a0 > 0:
            # If it is accelerating it should go straight
            vmin, vmax = v0 - v0*0.05, speed_limit
        elif a0 < 0:
            # If it is decelerating it should turn or stop
            vmin, vmax = v0*0.05, v0 + v0*0.05
        elif a0 == 0:
            # If it isn't accelerating or decelerating it could do anything
            vmin, vmax = v0*0.05, speed_limit

        # Compute the time range
        tmin, tmax = sf/vmax, sf/vmin

        # Compute the pass primitives
        stop_mp = stop_primitive(v0, a0, sf)
        pass_mp = pass_primitive(v0, a0, sf, vmin, vmax, tmin, tmax)

        # Get the most likely motion primitive that risks a collision
        if stop_mp and pass_mp:
            if 0 in pass_mp[0]['jerk_range']:
                coll_mp = pass_primitive_j0(v0, a0, sf, vmin, vmax)
            elif 0 in stop_mp['jerk_range']:
                pass
            elif min(abs(stop_mp['jerk_range'].extrema)) < min(abs(pass_mp[0]['jerk_range'].extrema)):
                pass
            else:
                coll_mp = pass_mp[np.argmin([abs(pass_mp[0]['coeffs'][3]), abs(pass_mp[1]['coeffs'][3])])]
        elif stop_mp:
            pass
        elif pass_mp:
            if 0 in pass_mp[0]['jerk_range']:
                coll_mp = pass_primitive_j0(v0, a0, sf, vmin, vmax)
            else:
                coll_mp = pass_mp[np.argmin([abs(pass_mp[0]['coeffs'][3]), abs(pass_mp[1]['coeffs'][3])])]

        # Save the collision if exists
        if coll_mp:
            # Compute the velocity and time ranges for the ego-vehicle for pass
            # before or after primitives
            a_tmin = [a_sf/a_speed_limit, coll_mp['tf']]
            a_tmax = [coll_mp['tf'],      a_sf/(a_v0*0.05)]
            a_vmin = [a_sf/coll_mp['tf'], a_v0*0.05]
            a_vmax = [a_speed_limit,      a_sf/coll_mp['tf']]

            # Append initial condition
            ego_init.append([a_v0, a_a0, a_sf, a_vmin, a_vmax, a_tmin, a_tmax])

            # Append collision trajectories
            coll_trajs.append([a_traj, traj])

    # Return the best mp
    return mp_as(ego_init), coll_trajs

# Compute the motion primitives and output the best action to take
def mp_as(conditions):
    # Initialise ranges and containers
    stop_jrange = interval([-inf, inf])
    pass_jrange = interval([-inf, inf])
    v_range     = interval([-inf, inf])
    mps         = []
    best_mp     = []
    pass_mp     = []

    # For each condition
    for v0, a0, sf, vmin, vmax, tmin, tmax in conditions:
        # Intervals for each condition
        j_cond = interval()
        v_cond = interval()

        # Compute the stop primitive
        stop_mp = stop_primitive(v0, a0, sf)

        # Check if there is a stop primitive
        if stop_mp:
            stop_jrange = stop_jrange & stop_mp['jerk_range']
            mps.append(stop_mp)

        # Compute both pass before and pass after primitives
        for v_min, v_max, t_min, t_max in zip(vmin, vmax, tmin, tmax):
            # Compute the pass primitives
            pass_mp = pass_primitive(v0, a0, sf, v_min, v_max, t_min, t_max)

            # Check if there is a pass primitive
            if pass_mp:
                j_cond = j_cond | pass_mp[0]['jerk_range']
                v_cond = v_cond | interval([v_min, v_max])
                mps.append(pass_mp[0])
                mps.append(pass_mp[1])

        # Intersect the intervals of each condition
        pass_jrange = pass_jrange & j_cond
        v_range     = v_range & v_cond

    # Check if there are mps
    if mps:
        # First check wether j0 = 0 is in the pass jerk range
        if 0 in pass_jrange and len(v_range) != 0:
            best_mp = pass_primitive_j0(v0, a0, sf, v_range[0].inf, v_range[0].sup)
        elif 0 in stop_jrange and 0 not in pass_jrange:
            best_mp = stop_primitive_j0(v0, a0)
        else:
            # Create total jerk range
            j_range = stop_jrange | pass_jrange

            for mp in mps:
                # Make sure best_mp is initialised
                if not best_mp:
                    best_mp = mp

                # The best motion primitive is the one with the minimum jerk and
                # that is in the admissible jerk range
                elif mp['coeffs'][3] in j_range and abs(mp['coeffs'][3]) <= abs(best_mp['coeffs'][3]):
                    best_mp = mp

    if best_mp:
        return ocp_sol(best_mp['coeffs'], best_mp['tf']), best_mp['type']
    else:
        return np.empty(0), None
