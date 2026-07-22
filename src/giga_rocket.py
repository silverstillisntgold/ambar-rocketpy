import datetime

from rocketpy.motors import CylindricalTank, Fluid, HybridMotor
from rocketpy.motors.tank import MassFlowRateBasedTank

import controller
from rocketpy import Environment, Flight, Rocket


def build_and_run_flight(
    controller_function,
    target_apogee: float,
    angle: float,
    accel,
    gyro,
    baro,
):
    flight_date = datetime.date(2024, 8, 24)
    env = Environment(latitude=47.966527, longitude=-81.87413, elevation=1383.4)
    env.set_date((flight_date.year, flight_date.month, flight_date.day, 0))
    env.set_atmospheric_model(type="custom_atmosphere", wind_v=1, wind_u=-2)
    oxidizer_liq = Fluid(name="N2O_l", density=960)
    oxidizer_gas = Fluid(name="N2O_g", density=1.9277)
    tank_shape = CylindricalTank(0.0665, 1.79)
    oxidizer_tank = MassFlowRateBasedTank(
        name="oxidizer_tank",
        geometry=tank_shape,
        flux_time=(6.5),
        liquid=oxidizer_liq,
        gas=oxidizer_gas,
        initial_liquid_mass=17,
        initial_gas_mass=0,
        liquid_mass_flow_rate_in=0,
        liquid_mass_flow_rate_out=(17 - 1e-7) / 6.5,
        gas_mass_flow_rate_in=0,
        gas_mass_flow_rate_out=0,
    )
    hybrid_motor = HybridMotor(
        thrust_source="../RocketPy/data/rockets/defiance/Thrust_curve.csv",
        dry_mass=13.832,
        dry_inertia=(1.801, 1.801, 0.0305),
        center_of_dry_mass_position=780 / 1000,
        reshape_thrust_curve=False,
        grain_number=1,
        grain_separation=0,
        grain_outer_radius=0.0665,
        grain_initial_inner_radius=0.061,
        grain_initial_height=1.25,
        grain_density=920,
        nozzle_radius=0.0447,
        throat_radius=0.0234,
        interpolation_method="linear",
        grains_center_of_mass_position=0.377,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )
    hybrid_motor.add_tank(tank=oxidizer_tank, position=2.2)
    defiance = Rocket(
        radius=0.07,
        mass=37.211,
        # inertia = (180.142, 180.142, 0.262),
        inertia=(94.14, 94.14, 0.09),
        center_of_mass_without_motor=3.29,
        power_off_drag="../RocketPy/data/rockets/defiance/DragCurve.csv",
        power_on_drag="../RocketPy/data/rockets/defiance/DragCurve.csv",
        coordinate_system_orientation="tail_to_nose",
    )

    avionics_position = defiance.center_of_mass_without_motor
    defiance.add_sensor(accel, position=avionics_position)
    defiance.add_sensor(gyro, position=avionics_position)
    defiance.add_sensor(baro, position=avionics_position)

    _air_brakes = defiance.add_air_brakes(
        drag_coefficient_curve=controller.AIRBRAKE_FILE,
        controller_function=controller_function,
        sampling_rate=controller.SAMPLING_RATE,
        reference_area=None,
        clamp=True,
        initial_observed_variables=[
            float(hybrid_motor.burn_out_time),
            float(target_apogee),
            0.0,
        ],
        override_rocket_drag=False,
        name="Air Brakes",
    )
    defiance.add_motor(hybrid_motor, position=0.2)
    defiance.add_nose(length=0.563, kind="vonKarman", position=4.947)
    defiance.add_trapezoidal_fins(
        n=3, span=0.115, root_chord=0.4, tip_chord=0.2, position=0.175
    )
    defiance.add_tail(top_radius=0.07, bottom_radius=0.064, length=0.0597, position=0.1)
    flight = Flight(
        rocket=defiance,
        environment=env,
        rail_length=10,
        inclination=angle,
        max_time=100,
        terminate_on_apogee=True,
        time_overshoot=False,
    )
    return flight, env
