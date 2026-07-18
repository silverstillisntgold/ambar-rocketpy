import argparse

import big_boy_rocket_frfr
import giga_rocket
import pointmass_rocket
import qt_wittle_wocket
from controller import controller_0, controller_1, controller_function

DEFAULT_ANGLE_FROM_HORIZONAL = 85.0
METERS_TO_FEET = 3.281
TYPES = {
    "pointmass": pointmass_rocket.build_and_run_flight,
    "wittle": qt_wittle_wocket.build_and_run_flight,
    "GIGA": giga_rocket.build_and_run_flight,
    "big_boi": big_boy_rocket_frfr.build_and_run_flight,
}

parser = argparse.ArgumentParser()
parser.add_argument("--angle", type=float, required=False)
parser.add_argument("--apogee", type=float, required=False)
parser.add_argument("--traj", action="store_true")
parser.add_argument("--type", choices=TYPES.keys(), required=True)
args = parser.parse_args()
launch_angle = args.angle if args.angle is not None else DEFAULT_ANGLE_FROM_HORIZONAL
target_apogee_agl = args.apogee
show_trajectory = args.traj
rocket_builder = TYPES[args.type]

if target_apogee_agl is None:
    print("Running no airbrake sim...", end="", flush=True)
    flight0, env0 = rocket_builder(
        controller_function=controller_0,
        target_apogee=0.0,
        angle=launch_angle,
    )
    print("done")
    print("Running full airbrake sim...", end="", flush=True)
    flight1, env1 = rocket_builder(
        controller_function=controller_1,
        target_apogee=0.0,
        angle=launch_angle,
    )
    print("done")
    max_apogee = float(flight0.apogee) - float(env0.elevation)
    min_apogee = float(flight1.apogee) - float(env1.elevation)
    print(f"Max apogee agl atainable: {max_apogee:.2f} meters")
    print(f"Min apogee agl atainable: {min_apogee:.2f} meters")
    print(f"Delta: {max_apogee - min_apogee:.2f} meters")
else:
    print("Running targeted airbrake sim...", end="", flush=True)
    flight, env = rocket_builder(
        controller_function=controller_function,
        target_apogee=target_apogee_agl,
        angle=launch_angle,
    )
    print("done")
    apogee_agl = float(flight.apogee) - float(env.elevation)
    apogee_dt_m = abs(target_apogee_agl - apogee_agl)
    apogee_dt_ft = apogee_dt_m * METERS_TO_FEET
    print(f"Target apogee agl: {target_apogee_agl:.2f} meters")
    print(f"Actual apogee agl: {apogee_agl:.2f} meters")
    print(f"Apogee delta: {apogee_dt_m:.2f} meters")
    print(f"Apogee delta: {apogee_dt_ft:.2f} feet (should be <=100.00 feet)")
    if show_trajectory:
        flight.plots.trajectory_3d()
        flight.info()
