import math

# Controls which airbrake profile is used.
AIRBRAKE_FILE = "air_brakes_cd.csv"

# How many seconds we wait after burnout.
POST_BURNOUT_TIME = 0.1
# The vertical velocity at which we start retracting the airbrakes.
VELOCITY_CUTOFF = 5.0
# Polling rate.
SAMPLING_RATE = 100.0
# Max change in % deployment per second.
MAX_CHANGE_PER_SECOND = 1.0
# Max change in % deployment per controller_function call.
MAX_CHANGE = MAX_CHANGE_PER_SECOND / SAMPLING_RATE
# Gravity type shit.
G = 9.80665
# 10 meters.
KP = 1.0 / 1000.0


def controller_0(
    time,
    sampling_rate,
    state,
    state_history,
    observed_variables,
    air_brakes,
    sensors,
    environment,
):
    """
    Controller which always keeps the airbrakes fully retracted.
    Assumes clamp=True.
    """
    air_brakes.deployment_level -= MAX_CHANGE
    return None


def controller_1(
    time,
    sampling_rate,
    state,
    state_history,
    observed_variables,
    air_brakes,
    sensors,
    environment,
):
    """
    Controller which always keeps the airbrakes fully extended after burnout.
    Assumes clamp=True.
    """
    motor_burn_out_time = observed_variables[0][0]
    if time < (motor_burn_out_time + POST_BURNOUT_TIME):
        air_brakes.deployment_level -= MAX_CHANGE
    else:
        air_brakes.deployment_level += MAX_CHANGE
    return None


def controller_function(
    time,  # Timekeeping (who would've guessed)
    sampling_rate,  # Not used
    state,  # For current state
    state_history,  # Not used
    observed_variables,  # For storing previous state
    air_brakes,  # For controlling deployment
    sensors,  # Not used
    environment,  # For elevation
):
    """
    Controller which is generic across airbrakes/rockets.
    Assumes clamp=True.
    """
    # We are using exact data from simulation, but good approximations
    # from high-quality filters should be a good enough substitute.
    # Me when the approximations aren't good enough *fucking dies*.
    vertical_velocity = state[5]
    altitude_agl = state[2] - environment.elevation
    # Janky approach but such is life in python.
    motor_burn_out_time = observed_variables[0][0]
    target_apogee_agl = observed_variables[0][1]

    # Too early in flight or descending, trend towards being closed.
    if (time < (motor_burn_out_time + POST_BURNOUT_TIME)) or (
        vertical_velocity <= VELOCITY_CUTOFF
    ):
        air_brakes.deployment_level -= MAX_CHANGE
        return (0.0, time, vertical_velocity)

    # DON'T USE 'state_history' IT'S FUCKED!!
    _discard, prev_time, prev_vertical_velocity = observed_variables[-1]
    # We have to compute this because for some fucking reason RocketPy
    # doesn't just give it to us directly? Dickheads.
    dv = vertical_velocity - prev_vertical_velocity
    dt = time - prev_time  # Never zero (anymore...)
    if dt < 1e-7:
        raise Exception("dt is 0.0 again fuck me")
    vertical_acceleration = dv / dt

    # """"1D"""" equation of motion for a coasting rocket :)
    vert_velo_2 = vertical_velocity * vertical_velocity
    # Instantaneous aerodynamic drag factor.
    k = -(vertical_acceleration + G) / vert_velo_2
    if k > 0.0:
        # Analytical solution to 1D equation of rocket motion.
        delta_h = math.log1p(k * vert_velo_2 / G) / (2 * k)
    else:
        # Kinematic fallback if we're fucked.
        delta_h = vert_velo_2 / (2 * G)

    # Calculate error and desired change.
    predicted_apogee = altitude_agl + delta_h
    error = predicted_apogee - target_apogee_agl
    desired_change = KP * error  # Smoothing.
    actual_change = max(-MAX_CHANGE, min(MAX_CHANGE, desired_change))
    air_brakes.deployment_level += actual_change
    return (0.0, time, vertical_velocity)
