# First-extension log and fixed-reference restart

2026-10-13. **[ADDED] Opt-in mode; original XR Delta is unchanged.**

## Saved user trial: do not pool later movements

Source: `ai_worker/docker/workspace/sg2_arm_logs/run_20261013_131120/`
(`arms.csv`, `events.jsonl`, `metadata.json`). The directory's name is not
the experiment start clock; data timestamps place this recording at
13:22:39 UTC / 22:22:39 Korea time. Original data were not edited.

First requested activation: elapsed3.6094s; first automatic stop:12.5876s.
For the initial reach only, analyze8.2 <= t <12.0. For the end hold, use
11.0 <= t <12.0 (10 paired samples; CSV lines220-239). Renewed upward target
motion appears around12.0-12.2s, so t>=12.0 is conservatively excluded.
The first reach itself includes forward/upward displacement; the CSV cannot
label exact human intent or provide a video-defined gesture boundary.

| First end-hold median | Left | Right |
|---|---:|---:|
| measured joint4 | -62.155deg | -65.807deg |
| commanded joint4 | -61.954deg | -65.454deg |
| margin to controller upper -40.107deg | 22.048deg | 25.700deg |
| wrist position error | 5.983mm | 13.455mm |
| wrist orientation error | 0.693deg | 0.393deg |
| published shoulder-to-goal distance | 0.607936m | 0.607936m |

The initial hold is NOT upper-limit saturation. The plateau matches the
existing approximate0.608m geometric shell. The old log lacks raw/pre-projection
requests, so the amount of clipping cannot be measured directly.
The whole first reach was still within the original controller's3s delay+8s
slow-start; its transient errors (about70/98mm peaks) must not be described
as settled tracking performance. First command was observed at6.7079s,
about3.0985s after the requested activation.

Automatic stop reasons were confirmed in the matching Vuer process log:
`robotis-applications:/root/.ros/log/python3_2646_1791897714335.log`.
Stops at12.5876,25.2271,48.5554s are all
`operator exceeded 0.5m clutch range; stop and recapture`.
The stop at77.1653s is Y+B. This run has no recorded reference-checker jump.

At subsequent new anchor frames (20.2083,27.6078,62.4079s), goal-versus-measured
position errors are below0.001mm, not a growing leftover EE error. The old
mapper is freshly re-zeroed at every X+A: if hands are moved while stopped,
their correspondence to robot position changes. Heading and approximate
shell dimensions are also recaptured. That is intentional reclutch semantics,
not evidence of a numerical additive-offset variable. Raw human poses are
absent, so perceived offset cannot be quantitatively reconstructed.

Numerical summary with source SHA256s:
`~/Downloads/.retarget-review/restart_20261013/first_extension_report.json`.

## New policy: X+A resumes, not reclutches

Use `run_sg2_xr_resume_sim.sh` for the new mode. Keep the old
`run_sg2_xr_sim.sh` for the previous reclutch behavior.

- First X+A creates one session: fixed heading, human/robot wrist anchors and
  radial shell. Subsequent stops and failed starts do not redefine these.
- On stop, discard transient goals, filter state, input history, active elbow
  continuation, command receipts and calibration feedback cache.
- To resume, require new increasing post-stop measurement timestamps over a
  stable0.30s window, fresh TF, matching inactive Cyclo FK, and BOTH hands'
  **unprojected** mapped targets within20mm/10deg of PhysX measured wrists.
  Raw goals must be in the original shell. Clipping does not count as alignment.
- Wrong-position X+A reports `ALIGN_REQUIRED` and publishes no new goal.
  Return hands to the stopped robot's ORIGINAL mapped location; do not expect
  the starting comfortable posture to align when the robot is still extended.
- Preserve last-published goals and reject a seed discontinuity before it can
  change reference-checker history. A reference-checker True is latched even
  when stopped; X+A and a synthetic False cannot acknowledge it.
- Hold stationary target poses through startup. Require exactly one expected
  `vr_controller` command producer for each arm; observe bilateral commands
  no earlier than3s after enable, then hold through8s of ramp. Also check live
  measured alignment throughout. Do not move hands until `[XR RESUME READY]`.
  This is an output-observation gate, not a new C++ active-state message.
- No joint trajectory is published by the new Vuer node. Existing joint,
  collision, speed, 0.5m range, 0.25m/90deg tracking and reference checks remain.
  The elbow upper bound and radial shell are NOT expanded by this change.

An explicit NEW calibration requires stopping and restarting the publisher;
after simulation reset, clock/frame discontinuity, mode change or reference
fault, inspect the cause and restart the Cyclo controller too. No automatic
home movement, safety reset or hidden remapping is performed.

## Run: changed ROBOTIS Applications command only

Keep existing SG2 simulator and AI Worker shoulder_free launch settings.
Stop old Vuer first. On changing modes, restart Cyclo with its existing command
to avoid carrying old reference-checker history into a new session.

Inside ROBOTIS Applications Docker:

```bash
export ROS_DOMAIN_ID=231
bash /root/ros2_ws/src/robotis_applications/run_sg2_xr_resume_sim.sh
```

First X+A calibrates; wait for `[XR RESUME READY]`. Y+B stops. After a range
stop, align hands using `[ALIGN]` residuals, release X/A, and press X+A again.
`hand_correction_robot_xyz_mm` is the translation correction in robot axes:
+X forward, +Y left, +Z up. Orientation must also be aligned within10deg.
The original mapping does not change while those numbers are displayed.

## Logging

Existing logging command is unchanged. The updated passive script was copied
to `ai_worker:/workspace/sg2_diagnostics/log_sg2_arm.py`; its prior copy is
`log_sg2_arm.before_fixed_resume.py`. Stop/restart logger into a NEW output
directory to use the extended CSV fields.

Added `/sg2_xr/events` and10Hz `/sg2_xr/diagnostics` record exact stop reason,
phase, session identifier, restart count, unprojected request, projection
distance, shell radius and alignment residual. CSV `xr_projection_mm` is
populated only with fresh diagnostics; legacy XR still runs with these
optional fields empty. Do not mistake WARMUP's requested activation for READY.

## Verification on this PC

- 45 selected unit tests passed: original20 math/ROS guards,4 controller-config
  preservation tests,21 new fixed-reference math/state/timing tests.
-Final real ROS2/Cyclo/PhysX test in domain232 passed11 integration checks.
  Input was SYNTHETIC WebXR callbacks, not replayed user/Quest data.
-Reproduced the unchanged0.5m range stop; an X+A about90mm away from the
  original mapping was refused without goal publication; aligning and
  restarting twice kept the calibration signature and shell unchanged.
-Same human input after repeated resumes mapped to exactly the same target
  (computed drift0m). Initial4cm synthetic reach tracking error was1.984mm.
  This is mapping invariance plus one simulated fixture, not zero physical
  tracking error for arbitrary teleop motions.
-Passive logger optional telemetry smoke passed. Original XR source hashes
  were unchanged; no controller/URDF/SRDF/YAML or saved user log was modified.

Final raw integration data:
`docker/workspace/xr_resume_release.json` in ai_worker_teleop_pc.
Logs: `~/Downloads/.retarget-review/restart_20261013/release/`.
No git commit/push was performed for this change.
