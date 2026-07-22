from dataclasses import dataclass

import numpy as np

ACCELEROMETER_NAME = "LSM6DSV32X Accelerometer"
GYROSCOPE_NAME = "LSM6DSV32X Gyroscope"
BAROMETER_NAME = "BMP388 Barometer"

# Set this to -1.0 if the physical accelerometer's positive axial direction is
# opposite RocketPy's positive rocket-axis direction.
AXIAL_ACCELERATION_SIGN = 1.0

# Pressure-to-relative-altitude constants. The launch-site temperature is
# supplied to the estimator instead of assuming the ISA sea-level value because
# otherwise shit is mega fucked.
LAPSE_K_PER_M = 0.0065
PRESSURE_EXPONENT = 0.190_263_24

# Filter tuning values. Acceleration is changes quickly through motor ignition/burnout,
# while the accelerometer bias is expected (like it's very unlikely it doesn't) to drift slowly.
JERK_NOISE_STD = 40.0  # m/s^3 / sqrt(Hz)
ACCEL_BIAS_WALK_STD = 0.02  # m/s^2 / sqrt(s)
ACCEL_MEASUREMENT_STD = 0.35  # m/s^2
BARO_ALTITUDE_STD = 0.25  # m

_TIME_EPSILON = 1e-9


@dataclass(frozen=True)
class VerticalEstimate:
    altitude_agl: float
    vertical_velocity: float
    vertical_acceleration: float


def relative_altitude_m(
    pressure_pa: float,
    baseline_pa: float,
    baseline_temperature_k: float,
) -> float:
    """Approximate altitude above the pressure baseline in meters."""
    if pressure_pa <= 0.0 or baseline_pa <= 0.0:
        raise ValueError("Pressure values must be positive")
    if baseline_temperature_k <= 0.0:
        raise ValueError("Baseline temperature must be positive")

    ratio = pressure_pa / baseline_pa
    return (baseline_temperature_k / LAPSE_K_PER_M) * (
        1.0 - np.power(ratio, PRESSURE_EXPONENT)
    )


class VerticalEstimator:
    """Asynchronous 1-D Kalman filter for altitude, velocity and acceleration.

    State: [altitude_agl, vertical_velocity, vertical_acceleration, accel_bias]

    The first accelerometer sample is used as the zero-acceleration baseline.
    This removes the stationary gravity reading and constant sensor offset
    without requiring a particular accelerometer sign convention from RocketPy.
    Can probably handle this in the real thing by just vibin on the pad for a bit.

    Pad pressure and launch-site temperature are supplied explicitly so that the
    pressure conversion does not assume ISA sea-level temperature because otherwise
    that shit just completely ruins the simulation accuracy (cringe).
    """

    def __init__(
        self,
        pressure_baseline: float,
        baseline_temperature_k: float,
    ):
        if pressure_baseline <= 0.0:
            raise ValueError("Pressure baseline must be positive")
        if baseline_temperature_k <= 0.0:
            raise ValueError("Baseline temperature must be positive")

        self.x = np.zeros(4, dtype=float)
        self.P = np.diag([1.0, 25.0, 100.0, 0.25]).astype(float)

        self.filter_time = None
        self.accelerometer_baseline = None
        self.pressure_baseline = float(pressure_baseline)
        self.baseline_temperature_k = float(baseline_temperature_k)

        self.accelerometer_index = 0
        self.barometer_index = 0

    def update(self, time: float, sensors) -> VerticalEstimate:
        sensor_by_name = {sensor.name: sensor for sensor in sensors}
        try:
            accelerometer = sensor_by_name[ACCELEROMETER_NAME]
            barometer = sensor_by_name[BAROMETER_NAME]
        except KeyError as error:
            raise RuntimeError(
                "Vertical estimator requires the configured accelerometer/barometer you fucking chud"
            ) from error

        events = []

        accelerometer_data = self._single_mount_data(accelerometer)
        while self.accelerometer_index < len(accelerometer_data):
            row = accelerometer_data[self.accelerometer_index]
            sample_time, _ax, _ay, az = row
            if sample_time > time + _TIME_EPSILON:
                break
            events.append((float(sample_time), 0, "accelerometer", float(az)))
            self.accelerometer_index += 1

        barometer_data = self._single_mount_data(barometer)
        while self.barometer_index < len(barometer_data):
            row = barometer_data[self.barometer_index]
            sample_time, pressure_pa = row
            if sample_time > time + _TIME_EPSILON:
                break
            events.append((float(sample_time), 1, "barometer", float(pressure_pa)))
            self.barometer_index += 1

        events.sort(key=lambda event: (event[0], event[1]))
        for sample_time, _priority, kind, value in events:
            if sample_time > time + _TIME_EPSILON:
                continue

            self._predict_to(sample_time)
            if kind == "accelerometer":
                self._process_accelerometer(value)
            else:
                self._process_barometer(value)

        self._predict_to(float(time))
        return VerticalEstimate(
            altitude_agl=float(self.x[0]),
            vertical_velocity=float(self.x[1]),
            vertical_acceleration=float(self.x[2]),
        )

    @staticmethod
    def _single_mount_data(sensor):
        data = sensor.measured_data
        if data and isinstance(data[0], list):
            raise RuntimeError(
                f"Sensor {sensor.name!r} is mounted more than once; "
                "the vertical estimator requires one mounting"
            )
        return data

    def _predict_to(self, sample_time: float) -> None:
        if self.filter_time is None:
            self.filter_time = sample_time
            return

        dt = sample_time - self.filter_time
        if dt <= _TIME_EPSILON:
            return

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        dt5 = dt4 * dt

        F = np.array(
            [
                [1.0, dt, 0.5 * dt2, 0.0],
                [0.0, 1.0, dt, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        jerk_variance = JERK_NOISE_STD * JERK_NOISE_STD
        Q = np.zeros((4, 4), dtype=float)
        # got your weiner lol
        Q[:3, :3] = jerk_variance * np.array(
            [
                [dt5 / 20.0, dt4 / 8.0, dt3 / 6.0],
                [dt4 / 8.0, dt3 / 3.0, dt2 / 2.0],
                [dt3 / 6.0, dt2 / 2.0, dt],
            ],
            dtype=float,
        )
        Q[3, 3] = ACCEL_BIAS_WALK_STD * ACCEL_BIAS_WALK_STD * dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)
        self.filter_time = sample_time

    def _process_accelerometer(self, axial_measurement: float) -> None:
        if self.accelerometer_baseline is None:
            self.accelerometer_baseline = axial_measurement
            return

        measured_acceleration = AXIAL_ACCELERATION_SIGN * (
            axial_measurement - self.accelerometer_baseline
        )
        H = np.array([0.0, 0.0, 1.0, 1.0], dtype=float)
        self._update_scalar(
            measured_acceleration,
            H,
            ACCEL_MEASUREMENT_STD * ACCEL_MEASUREMENT_STD,
        )

    def _process_barometer(self, pressure_pa: float) -> None:
        measured_altitude = relative_altitude_m(
            pressure_pa,
            self.pressure_baseline,
            self.baseline_temperature_k,
        )
        H = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self._update_scalar(
            measured_altitude,
            H,
            BARO_ALTITUDE_STD * BARO_ALTITUDE_STD,
        )

    def _update_scalar(
        self, measurement: float, H: np.ndarray, variance: float
    ) -> None:
        innovation = measurement - float(H @ self.x)
        innovation_variance = float(H @ self.P @ H + variance)
        kalman_gain = (self.P @ H) / innovation_variance

        self.x = self.x + kalman_gain * innovation

        identity = np.eye(4)
        correction = identity - np.outer(kalman_gain, H)
        self.P = (
            correction @ self.P @ correction.T
            + np.outer(kalman_gain, kalman_gain) * variance
        )
        self.P = 0.5 * (self.P + self.P.T)
