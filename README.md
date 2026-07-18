# ambar-rocketpy

Requires the latest dev build of [`RocketPy`] (included as a sub-repo). The current release build contains a bug
in the controller backend which causes finite differentation of timestamps (used to find acceleration) to fail.

To ensure you're using the latest dev build, clone this repo recursively, then navigate to the local [`RocketPy`] repo and run:

```bash
py -m pip uninstall rocketpy
py -m pip install ".[all]"
```

Ensure you're using the python install manager and have the latest python3 version installed.

[`RocketPy`]: https://github.com/RocketPy-Team/RocketPy
