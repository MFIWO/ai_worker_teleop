# Offline shutdown dependency

Unmodified pyserial 3.5 wheel from PyPI; the BSD license is included inside
the wheel. Used only by the robot exit helper with the installed ROS
DynamixelSDK. Installed under `/workspace/ai_worker_xr/vendor`, not into the
system or native driver environment.

SHA256: `c4451db6ba391ca6ca299fb3ec7bae67a5c55dde170964c7a14ceefec02f2cf0`

Source: https://pypi.org/project/pyserial/3.5/

## HX5 xr_ai_teleop mode 2

Pinned pure Python wheels: anytree 2.12.1, trimesh 4.4.9, pytransform3d 3.14.4.
Their license files are included in each wheel. Installed with `--no-deps`
under `/workspace/ai_worker_xr/vendor`; existing numpy, torch, Pinocchio,
nlopt and ROS packages are retained. The dex-retargeting source is vendored in
`vendor/dex_retargeting` with its MIT license; the revision and xr_ai_teleop
adapter provenance are in `hardware/xr_ai_assets/provenance.json`.
