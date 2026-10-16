# ambar-rocketpy

A repo for experimenting with active airbrake control.

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

Those two values define the apogee window, and any height between them is a reasonable target.

From there you can test different apogee settings. The `--traj` argument writes an animation of the flight to
"flight_animation.mp4" in the root directory. The `--angle` settings lets you specifiy a launch angle from horizontal.

Python 3.15 can't be used when passing `--traj`, as the required pyvista backend used to generate the animation has
a dependency chain that doesn't support the latest Python version.

[`RocketPy`]: https://github.com/RocketPy-Team/RocketPy
