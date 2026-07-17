import controller
from rocketpy import Environment, Flight, Rocket, SolidMotor


def build_and_run_flight(controller_function, target_apogee: float, angle: float):
    env = Environment(
        latitude=27.933337901305062, longitude=-80.70898578225906, elevation=6.9
    )
    env.set_atmospheric_model(type="standard_atmosphere", wind_u=20.0, wind_v=-20.0)
    Pro75M1670 = SolidMotor(
        thrust_source="data/motors/cesaroni/Cesaroni_M1670.eng",
        dry_mass=1.815,
        dry_inertia=(0.125, 0.125, 0.002),
        nozzle_radius=33 / 1000,
        grain_number=5,
        grain_density=1815,
        grain_outer_radius=33 / 1000,
        grain_initial_inner_radius=15 / 1000,
        grain_initial_height=120 / 1000,
        grain_separation=5 / 1000,
        grains_center_of_mass_position=0.397,
        center_of_dry_mass_position=0.317,
        nozzle_position=0,
        burn_time=3.9,
        throat_radius=11 / 1000,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )
    calisto = Rocket(
        radius=127 / 2000,
        mass=14.426,
        inertia=(6.321, 6.321, 0.034),
        power_off_drag="data/rockets/calisto/powerOffDragCurve.csv",
        power_on_drag="data/rockets/calisto/powerOnDragCurve.csv",
        center_of_mass_without_motor=0,
        coordinate_system_orientation="tail_to_nose",
    )
    _air_brakes = calisto.add_air_brakes(
        drag_coefficient_curve=controller.AIRBRAKE_FILE,
        controller_function=controller_function,
        sampling_rate=controller.SAMPLING_RATE,
        reference_area=None,
        clamp=True,
        initial_observed_variables=[
            float(Pro75M1670.burn_out_time),
            float(target_apogee),
            0.0,
        ],
        override_rocket_drag=False,
        name="Air Brakes",
    )
    calisto.add_motor(Pro75M1670, position=-1.255)
    _rail_buttons = calisto.set_rail_buttons(
        upper_button_position=0.0818,
        lower_button_position=-0.6182,
        angular_position=45,
    )
    _nose_cone = calisto.add_nose(length=0.55829, kind="von karman", position=1.278)
    _fin_set = calisto.add_trapezoidal_fins(
        n=4,
        root_chord=0.120,
        tip_chord=0.060,
        span=0.110,
        position=-1.04956,
        cant_angle=0.5,
        airfoil=("data/airfoils/NACA0012-radians.txt", "radians"),
    )
    _tail = calisto.add_tail(
        top_radius=0.0635, bottom_radius=0.0435, length=0.060, position=-1.194656
    )
    flight = Flight(
        rocket=calisto,
        environment=env,
        rail_length=5.2,
        inclination=angle,
        max_time=100,
        terminate_on_apogee=True,
        time_overshoot=False,
    )
    return flight, env
