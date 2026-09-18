# Source provenance

- `original/`: ROBOTIS-GIT/cyclo_control at the deployed base `651aa55`.
- `files/`: matching deployment files with the VR-only launch route, Python entry point,
  dependencies, and final gripper command forwarding changes described in README.md.
- `files/cyclo_motion_controller_ros_py/scripts/arm_retargeting.py`: upstream
  ROBOTIS-GIT/cyclo_control commit `b1b2033965e654cab252d4b68b1719e7e6d65b80`.
- `robotis-applications` branch: ROBOTIS-GIT/robotis_applications history through
  `8d7fe8b`, followed by local Quest changes. Original SH5 publisher is unchanged.
- `loop/ffw_loop_streamer`: local Loop integration package (Apache-2.0 per its
  package.xml), including the VR profile and configurable source_key.

ROBOTIS source copyright and Apache-2.0 notices are retained in the source files;
see LICENSE. The original/ directory is an installation compatibility snapshot,
not a second runnable workspace. The installer refuses unrelated source changes.
