import controller
from rocketpy import Environment, Flight, PointMassMotor, PointMassRocket


def build_and_run_flight(controller_function, target_apogee: float, angle: float):
    env = Environment(
        latitude=27.933337901305062, longitude=-80.70898578225906, elevation=6.9
    )
    env.set_atmospheric_model(type="standard_atmosphere", wind_u=20, wind_v=-20)
    motor = PointMassMotor(
        thrust_source="../RocketPy/data/motors/cesaroni/Cesaroni_M1670.eng",
        dry_mass=1.815,
        propellant_initial_mass=2.5,
    )
    rocket = PointMassRocket(
        radius=0.0635,  # meters
        mass=5.0,  # kg (dry mass without motor)
        center_of_mass_without_motor=0.0,
        power_off_drag=0.5,  # Constant drag coefficient
        power_on_drag=0.5,
    )
    _air_brakes = rocket.add_air_brakes(
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
    rocket.add_motor(motor, position=0)
    flight = Flight(
        rocket=rocket,
        environment=env,
        rail_length=5.2,
        inclination=angle,
        simulation_mode="3 DOF",
        max_time=100,
        terminate_on_apogee=True,
        time_overshoot=False,
    )
    return flight, env
