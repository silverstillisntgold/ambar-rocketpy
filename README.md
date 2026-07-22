# ambar-rocketpy

A repo for experimenting with vertical rocket state estimation and active airbrake control.

## Usage

Requires at least [`RocketPy`] v1.13.0 (included as a submodule). Older releases contained a bug
in the controller which was tremendously cringe to identify and work around.

The submodule is required for the rocket example data it provides.

You can ensure you have the latest [`RocketPy`] by running:

```bash
py -m pip uninstall rocketpy
py -m pip install rocketpy
```

Ensure you're using the python install manager and have the latest python3 version installed.

[`RocketPy`]: https://github.com/RocketPy-Team/RocketPy
