# ambar-rocketpy

A repo for experimenting with vertical rocket state estimation and active airbrake control.

## RocketPy

Requires at least [`RocketPy`] v1.13.0 (included as a submodule). Older releases contained a bug
in the controller which was tremendously cringe to identify and work around.

The submodule is required for the rocket example data it provides.

You can ensure you have the latest [`RocketPy`] by running:

```bash
py -m pip uninstall rocketpy
py -m pip install rocketpy
```

Ensure you're using the python install manager and have the latest python3 version installed.

## Usage

Navigate to the `src` directory and run `py sim.py --help`. There are currently four rocket options, and
they will all be listed there. Running the simulation with no specified apogee is the recommended first step,
as that will show you the min/max attainable apogee with/without full airbrake deployment after burnout.

From there you can test different apogee settings. The `--traj` setting writes an animation of the flight to
"flight_animation.mp4" in the root directory. The `--angle` settings lets you specifiy a launch angle from horizontal.

[`RocketPy`]: https://github.com/RocketPy-Team/RocketPy
