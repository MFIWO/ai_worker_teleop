# SG2 near-extension + fixed-calibration teleoperation experiment

Added 2026-10-13. Simulation only. Original XR Delta, fixed-resume publisher,
Cyclo source/config/model and all existing logs are preserved. No Git push.

## What H1_2/H2 actually compute

Pinned source: MFIWO/xr_tele main
`9faf1aafb81765896973527c6488f088252a2b50`,
`teleop/robot_control/robot_arm_ik.py`:

- H1_2, lines703–741: wrist-position/rotation costs, generic joint regularizer,
  continuity cost and URDF bounds. No explicit human elbow/full-extension target.
- H2, lines1291–1331: same structure, rotation coefficient0.8 instead of1.
- H1_2 position scaling1.25 and Unitree wrist+0.05m frames are NOT transferred.
- H1_2/H2 geometry and angle signs differ from SG2; q4=0 is not a transferable
  definition of a straight human or robot arm.

The experiment retains the existing wrist-centric Cyclo QP. It is not a port
of CasADi/IPOPT, and does not send direct elbow angles from Quest.

## Joint coordinate versus geometric bend

For SG2 actual controller URDF SHA256
`fdf2734725cbb5b7c2ebbf36d314b6e00dfc902ba8be331700df48284a84e063`,
take S=arm_link2 origin, E=arm_link4 origin, P=arm_link5 origin. The measured
chord bend is `atan2(norm(cross(E-S,P-E)), dot(E-S,P-E))`. Zero means those
three origins are collinear, not that the raw joint coordinate equals zero.

With d=0.041004m, the local vectors (apart from a common rotation) are:

```text
u = [d, 0, -0.300]
f = Ry(q4) * [-d, 0, -0.1489]
q4_collinear = -atan2(d,0.300) - atan2(d,0.1489)
             = -0.404557236717 rad = -23.179422 degrees
```

| joint4 coordinate | S2→E4→P5 bend |
|---|---:|
| -0.70rad (-40.107deg), original controller upper | 16.928deg |
| -0.45rad (-25.783deg), new experiment upper | 2.604deg |
| -0.404557rad | 0deg, chord collinearity |
| 0rad | 23.179deg, on the other side of chord collinearity |

This three-origin diagnostic is not the visual housing outline or a physical
certification of a mechanical stop. When the endpoint is wrist link7 instead,
q5/q6 matter. For q5=q6=0, S2/E4/W7 collinearity occurs at -0.255336495044rad
(-14.629704deg). Do not label these two different geometric definitions alike.
The logger writes both `elbow_proximal_bend_deg` and `elbow_wrist_bend_deg`.

## Why this experiment needs three separate changes

1. A COPY of the controller URDF changes only both joint4 upper bounds from
   -0.70 to -0.45rad. Other limits and all geometry remain identical. The wider
   description/USD bounds were not treated as permission to operate hardware.
2. A new publisher subtype expands its calibrated radial shell once, from the
   legacy0.94 fraction to0.995. Mapper, heading and that shell survive every
   pause/resume. The finite reach margin, input speed/jump/clutch checks, real
   measured startup/TF checks and reference-fault latch remain active.
3. An isolated core-library build tightens QP absolute/relative termination
   tolerances to1e-5 only when `CYCLO_SG2_QP_PRECISION=1`. Collision distances,
   CBF parameters, slack penalties, damping, startup and reference checker are
   not loosened. This is numeric solver precision, NOT a safety-distance value.

The generated config clones the FULL installed YAML. It keeps the already
tested shoulder-free elbow cost0 and wrist position/orientation weights10/1.
Installed Cyclo files are not overwritten. The precision build lives in
`/workspace/sg2_extension_overlay`; its source manifest records the original
QP header SHA256 `91f50be31975f173b6fcf797b9d9d851cc752d18ffca08580145c60a7140de33`.

## Trials and honest interpretation

All trials below used actual ROS2, Cyclo QP, SG2 PhysX and the passive logger,
with SYNTHETIC WebXR wrist poses in domain232. No direct fixture joint commands
were sent. A FK fixture gradually requested q1=-1.11, q4=-0.46, others0 only
to generate a reachable wrist pose; the QP was free to choose its own joints.

Earlier failed trials were kept:

- Expanded model/shell with original solver precision and position weight10:
  after3s hold, joint4≈-0.6546rad, proximal bend14.33deg, wrist error7.95mm.
  The near-straight test FAILED.
- Position weight50 alone (otherwise original precision): bend11.42deg at3s
  and11.41deg at8s, wrist error5.52mm. The near-straight test FAILED. This
  weight change was NOT selected for the released experiment.
- Restored weight10 plus isolated1e-5 precision core: bend4.12deg at3s and
  4.11deg at8s, joint4≈-0.476216rad, wrist error0.561mm/0.0094deg. Both arms
  passed all11 integration checks, including wrong-pose resume refusal,
  no goal publication on refusal and unchanged calibration+shell on resume.

These are not identical measurement durations: trial1 uses3s endpoint medians,
while later trials retain explicit3s values and8s endpoint medians. Full logs
are retained. An additional run of the fully original model/shell baseline
was blocked by the tool and was not completed; no result is claimed for it.

Nearest enabled collision distance in the inspected final-pose snapshots was
about0.109m, beyond the existing0.05m activation buffer. This is a snapshot,
not a proof that all extension trajectories or live Quest poses are collision
free. The improved result under tighter solve tolerances does not establish a
universal convergence rate or hardware accuracy.

Final log directory (host):

```text
~/Downloads/.retarget-review/extension_20261013/extension_precision/
~/Downloads/external_repos/ai_worker/docker/workspace/sg2_arm_logs/extension_extension_precision/
~/Downloads/external_repos/ai_worker_teleop_pc/docker/workspace/extension/extension_precision.json
```

The additional12.66deg wrist-chord bend in that test is intentional to report:
near-collinearity of S2/E4/P5 does not imply S2/E4/W7 is exactly collinear while
maintaining an arbitrary wrist orientation. This mode permits **near** extension;
it does not forcibly straighten a robot whose requested EE pose is incompatible.

## Commands — Isaac Sim unchanged

Stop old Vuer/VR controller first. Keep physical robot communication separate.
The default physics model already permits this simulated angle range; no new
Isaac Sim installation or bridge change is needed.

AI Worker Docker (script sources ROS and optional overlay only for its child):

```bash
export ROS_DOMAIN_ID=231
bash /workspace/sg2_xr_profiles/run_extension_controller.sh
```

ROBOTIS Applications Docker:

```bash
export ROS_DOMAIN_ID=231
bash /root/ros2_ws/src/robotis_applications/run_sg2_xr_extension_sim.sh
```

Logger in another AI Worker Docker terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=231 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
LOG_DIR="/workspace/sg2_arm_logs/extension_$(date +%Y%m%d_%H%M%S)"
python3 -u /workspace/sg2_diagnostics/log_sg2_arm.py \
  --urdf /workspace/sg2_xr_profiles/extension/sg2_near_extension.urdf \
  --output "$LOG_DIR" --hz 10
```

The explicit `--urdf` is essential: otherwise the passive logger reads the
original installed -0.70 limit instead of this experiment's -0.45 limit.
Log joint4, proximal/wrist bend, raw radius, projection amount, phase, session
ID and tracking errors. First X+A calibrates; HOLD until `[XR RESUME READY]`.
Then slowly extend forward with approximately constant wrist orientation,
hold, stop with Y+B and align before X+A resume. Misalignment must be refused
rather than resetting the mapping. Original0.5m clutch range is unchanged.

## Reproducibility / rollback

Generated files already exist on this PC:

```text
ai_worker:/workspace/sg2_xr_profiles/extension/{near_extension.yaml,sg2_near_extension.urdf,near_extension_manifest.json}
ai_worker:/workspace/sg2_extension_overlay/install/
```

Portable generation/build scripts are `sim/make_extension_profile.py` and
`sim/setup_extension_precision.sh`; copy into AI Worker Docker, source ROS,
set simulation domain231, generate the profile in a new output directory and
build the separate overlay. Generated outputs refuse unrelated overwrites.
The wrapper verifies `ldd` selects the overlay library. Publisher parameter
checks verify the live model filename/weights, not cached-model content after
`ros2 param set`; clean controller restart is required.

Rollback: close the two new processes and run the previous shoulder_free
controller and `run_sg2_xr_resume_sim.sh` in fresh terminals. The original
files were preserved, and the child-script environment does not alter the
parent shell. No physical-robot deployment, push or automatic activation occurs.

## 2026-10-14: X+A accepted but the simulated arms never moved

The real Quest recording `extension_20261013_153110` and matching Vuer logs
showed successful capture/reactivate, but no publisher ACTIVE/READY phase.
The first captures in three Vuer sessions stopped during WARMUP at a raw-hand
alignment error of right25.1mm, left20.9mm, and right20.2mm. An additional
resume stopped at left20.6mm. The old startup guard required20mm/10deg alignment
throughout the entire Cyclo3s pending plus8s ramp; small hand movement cancelled
the startup. The controller correctly held its starting pose instead of
following the hands. Valid paired goal/PhysX errors in the saved CSV were below
0.547mm. Two throttled QP non-convergence warnings also occurred in one session,
but the common recorded stop reason was the publisher's input-alignment guard,
not a recorded reference fault. No solver or collision setting was changed here.

**[FIX] Extension publisher only; the original XR/fixed-resume sources remain
byte-identical.** `run_sg2_xr_extension_sim.sh` is still the command to use.

- A first aligned X+A captures once, as before. Startup now prints
  `[XR STARTUP]` with phase, bilateral alignment errors and ramp progress.
- If only the human input drifts outside20mm/10deg during startup, the robot
  continues receiving the exact held goal pair. The immutable session and
  reference are not reset, and no human motion is queued or replayed.
- If any such drift occurred, completion enters `[XR RESUME WAIT_START]`.
  Bring both hands back to the ORIGINAL mapping, release X/A and press a fresh
  X+A to confirm. Returning the hands alone never enables motion. A wrong-pose
  confirmation logs `CONFIRM_REFUSED` and keeps the robot held.
- A correct confirmation checks original raw-input alignment against both held
  and measured wrist poses (20mm/10deg), fresh bilateral Cyclo commands,
  stable measured joints, model/graph/profile and shoulder consistency. It
  changes only phase to ACTIVE; no calibration or reactivate edge is generated.
- If the hands remain aligned throughout startup, the previous automatic READY
  path is retained. Wait for `[XR RESUME READY]` before testing extension.
- Measured robot drift, stale tracking/commands, hard reference faults, the
  original0.5m clutch limit, and joint4 experiment bound still STOP. Y+B stops
  all phases. Waiting for manual confirmation times out after60s, stopping
  without changing the session. Safety thresholds and ACTIVE retargeting are
  not relaxed.

Verification:62 XR tests plus4 config tests passed. The new15 tests exercise
20.2–25.1mm startup inputs, held-goal invariance, original mapping preservation,
manual-only entry, X/A release-edge semantics, measured reset and stale/fault
stops. A real Cyclo precision-overlay + PhysX run on test domain232 injected
25.1mm of early input movement, reached WAIT_START, refused wrong confirmation,
then accepted aligned X+A without another warmup. The same run extended both
arms to joint4≈-0.4764rad / proximal bend≈4.116deg with wrist error≈0.567mm,
and preserved calibration through the subsequent pause/resume. All18 checks
passed. This is synthetic-input verification, not a claim that the user has
already repeated the corrected flow with their Quest.

Evidence (original user logs untouched):

```text
~/Downloads/.retarget-review/no_motion_20261014/{vuer_attempt1,vuer_attempt2,vuer_attempt3,controller,checker}.log
~/Downloads/.retarget-review/no_motion_20261014/all_units.log
~/Downloads/.retarget-review/no_motion_20261014/integration_hold_confirm/
robotis-applications:/workspace/extension/extension_hold_confirm.json
ai_worker:/workspace/sg2_arm_logs/extension_extension_hold_confirm/
```

The preceding extension source and launcher are backed up as
`no_motion_20261014/extension_before.py` and `launcher_before.sh`. No git push.
