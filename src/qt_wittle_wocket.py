import numpy as np

import controller
from rocketpy import Environment, Flight, Rocket, SolidMotor


def build_and_run_flight(controller_function, target_apogee: float, angle: float):
    parameters = {
        # Mass Details
        "rocket_mass": (18.998, 0.010),  # Rocket dry mass: 20.846 kg
        # propulsion details
        "motor_structure_mass": (1.848, 0.1),
        "burn_time": (3.433, 0.1),
        "nozzle_radius": (0.02475, 0.001),
        "throat_radius": (0.01075, 0.001),
        "grain_separation": (0.003, 0.001),
        "grain_density": (1519.708, 30),
        "grain_outer_radius": (0.033, 0.001),
        "grain_initial_inner_radius": (0.015, 0.002),
        "grain_initial_height": (0.12, 0.001),
        "grains_center_of_mass_position": (-0.35, 0.100),
        "nozzle_position": (0, 0.100),
        "motor_position": (3.391, 0.100),
        # aerodynamic details
        "center_of_mass_without_motor": (1.3, 0.100),
        "drag_coefficient": (0.44, 0.1),
        "inertia_i": (73.316, 0.3 * 73.316),
        "inertia_z": (0.15982, 0.3 * 0.15982),
        "radius": (0.1015, 0.001),
        "power_off_drag": (1, 0.033),
        "power_on_drag": (1, 0.033),
        ## nose cone
        "nose_length": (0.610, 0.001),
        "nose_radius": (0.1015, 0.001),
        "nose_position": (0, 0.100),
        ## fins
        "fin_span": (0.165, 0.001),
        "fin_root_chord": (0.152, 0.001),
        "fin_tip_chord": (0.0762, 0.001),
        "fin_sweep_angle": (13, 0.5),
        "fin_position": (3.050, 0.100),
        ## transitions
        "transition_top_radius": (0.1015, 0.010),
        "transition_bottom_radius": (0.0775, 0.010),
        "transition_length": (0.127, 0.010),
        "transition_position": (1.2, 0.010),
        # launch and environment details
        "wind_direction": (0, 3),
        "wind_speed": (1, 0.30),
        "inclination": (90, 1),
        "heading": (181, 3),
        "rail_length": (3.353, 0.001),
        # parachute details
        "cd_s_drogue": (1.5 * np.pi * (24 * 25.4 / 1000) * (24 * 25.4 / 1000) / 4, 0.1),
        "cd_s_main": (2.2 * np.pi * (120 * 25.4 / 1000) * (120 * 25.4 / 1000) / 4, 0.1),
        "lag_rec": (1, 0.5),
    }
    # rocket: nose_to_tail
    # Environment conditions
    env = Environment(
        gravity=9.81,
        latitude=41.775447,
        longitude=-86.572467,
        date=(2020, 2, 23, 16),
        elevation=206,
    )
    env.set_atmospheric_model(
        type="Reanalysis",
        file="data/weather/ndrt_2020_weather_data_ERA5.nc",
        dictionary="ECMWF_v0",
    )
    motor = SolidMotor(
        thrust_source="data/motors/cesaroni/Cesaroni_4895L1395-P.eng",
        burn_time=parameters["burn_time"],
        dry_mass=parameters["motor_structure_mass"],
        dry_inertia=(0, 0, 0),
        center_of_dry_mass_position=parameters["grains_center_of_mass_position"],
        grains_center_of_mass_position=parameters["grains_center_of_mass_position"],
        grain_number=5,
        grain_separation=parameters["grain_separation"],
        grain_density=parameters["grain_density"],
        grain_outer_radius=parameters["grain_outer_radius"],
        grain_initial_inner_radius=parameters["grain_initial_inner_radius"],
        grain_initial_height=parameters["grain_initial_height"],
        nozzle_radius=parameters["nozzle_radius"],
        throat_radius=parameters["throat_radius"],
        interpolation_method="linear",
        nozzle_position=parameters["nozzle_position"],
        coordinate_system_orientation="combustion_chamber_to_nozzle",  # combustion_chamber_to_nozzle"
    )
    ndrt2020 = Rocket(
        radius=parameters["radius"],
        mass=parameters["rocket_mass"],
        inertia=(
            parameters["inertia_i"],
            parameters["inertia_i"],
            parameters["inertia_z"],
        ),
        power_off_drag=parameters["drag_coefficient"],
        power_on_drag=parameters["drag_coefficient"],
        center_of_mass_without_motor=parameters["center_of_mass_without_motor"],
        coordinate_system_orientation="nose_to_tail",
    )
    _air_brakes = ndrt2020.add_air_brakes(
        drag_coefficient_curve=controller.AIRBRAKE_FILE,
        controller_function=controller_function,
        sampling_rate=controller.SAMPLING_RATE,
        reference_area=None,
        clamp=True,
        initial_observed_variables=[
            float(motor.burn_out_time),
            float(target_apogee),
            0.0,
        ],
        override_rocket_drag=False,
        name="Air Brakes",
    )
    ndrt2020.set_rail_buttons(1.5, 2, 45)
    ndrt2020.add_motor(motor=motor, position=parameters["motor_position"])
    _nose_cone = ndrt2020.add_nose(
        length=parameters["nose_length"],
        kind="tangent",
        position=parameters["nose_position"],
    )
    _fin_set = ndrt2020.add_trapezoidal_fins(
        4,
        span=parameters["fin_span"],
        root_chord=parameters["fin_root_chord"],
        tip_chord=parameters["fin_tip_chord"],
        position=parameters["fin_position"],
        sweep_angle=parameters["fin_sweep_angle"],
        radius=parameters["transition_bottom_radius"],
    )
    _transition = ndrt2020.add_tail(
        top_radius=parameters["transition_top_radius"],
        bottom_radius=parameters["transition_bottom_radius"],
        length=parameters["transition_length"],
        position=parameters["transition_position"],
    )
    flight = Flight(
        rocket=ndrt2020,
        environment=env,
        rail_length=parameters["rail_length"],
        inclination=angle,
        max_time=100,
        terminate_on_apogee=True,
        time_overshoot=False,
    )
    return flight, env
