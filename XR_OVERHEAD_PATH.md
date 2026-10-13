# SG2 guided overhead / straight calibration-ray experiment

2026-10-14. **Simulation only.** Original XR/extension/guided modules, controller
library/model/config, and user recordings were preserved. No Git commit/push.

## User-observed stop: confirmed source

The latest full Vuer log is saved at:

`~/Downloads/.retarget-review/overhead_return_20261014/vuer.log`

Lines38-39 and67-68 record two ACTIVE stops at ROS wall seconds
1791912548.414 and1791912616.688. Both reasons are
`original 0.5m clutch range exceeded; return hand, do not reclutch`.
There was no separate above-the-head Z threshold in the inspected publisher.
This limits distance from the captured wrist, not height or robot arm length.

The newest CSV with the reused folder name `extension_20261013_153110` contains
only319 rows per arm / roughly32 seconds AFTER the last active stop. Its
samples are ALIGN_REQUIRED; it cannot establish the operator's earlier moving
return path. A snapshot is retained as `overhead_return_20261014/user_log/`.
Do not mix its current contents with an earlier recording bearing that name.

## Straight Cartesian motion versus changing joint posture

The raw mapper remains affine: `p_raw=p_robot0+s*B*(p_controller-p_controller0)`.
`B` is frozen at initial calibration. Straight positions stay straight in this
mapping, independently of the separately processed controller rotation.

Legacy `ContinuousElbowHint.project_wrist` scales the shoulder-relative vector:
`p_out=S+clip(||p_raw-S||,r_min,r_max)*(p_raw-S)/||p_raw-S||`.
Outside the shell this changes several Cartesian components, so even a straight
input can become a curved target. It is stateless: EXACTLY retraced raw inputs
also retrace the SAME projected curve. It alone does NOT explain directional
out/back hysteresis. Filtering/lag, wrist orientation changes and redundant
joint posture must be distinguished from this geometric distortion.

The current shoulder-free Cyclo cost uses wrist velocity tracking and velocity
damping. It does not require returning to the original seven-joint configuration.
The real test below confirms a nearly straight EE return while some joints
remain about6degrees from their initial values. This mode does NOT add a
posture-restoring/nullspace objective or lock shoulder yaw/roll/pitch.

Primary reference for the distinction between task and joint trajectories:
MathWorks, "Plan and Execute Task- and Joint-Space Trajectories Using Kinova Gen3
Manipulator", https://www.mathworks.com/help/robotics/ug/plan-and-execute-trajectory-kinova-gen3.html.
It is supplementary context, not evidence of the user's recorded motion.

## New opt-in behavior

`XROverheadPublisher` inherits `XRGuidedPublisher` and retains its passthrough
calibration display, fixed reference, startup confirmation, and manual stop.
It removes ONLY the arbitrary0.5m clutch stop in both ACTIVE and startup.

`AnchorRayHint` reuses the existing extension's measured shell and elbow hint.
It clips one scalar along the segment from the IMMUTABLE captured wrist:

`p_workspace=p_robot0+alpha*(p_raw-p_robot0), 0<=alpha<=1`.

The segment ends at its first exit from the permitted shell, including the
first entry into its inner hole. Thus a straight calibration ray out/back
remains on the same ray; at maximum reach it saturates rather than sliding
sideways on the sphere. This is NOT a proof of wrist-orientation reachability.
It also does not guarantee straightness for arbitrary offset strokes, which
can still be modified by a workspace boundary. Fully unconstrained linear
targets and guaranteed kinematic feasibility cannot both be promised.

After the existing speed/SO(3) filter, each finite previous-to-new step is
clipped along that same segment. Tangent-cone roots also prevent that segment
from entering the inner sphere's shadow relative to the saved anchor. This
keeps a commanded point compatible with the unchanged resume alignment rule.
No dynamic ray origin, new calibration, sphere radius, or head yaw is inferred.

Preserved: joint4 upper=-.45 in the already-existing sim model, other joint
limits, CBF/collision settings, QP precision overlay, startup/reference checks,
0.25m/90deg following-error stops, feedback/tracking validity and jump checks,
unique command-writer checks, X+A alignment, Y+B and Ctrl+C. This is not an
"all safety off" mode and must never be used in physical robot domain30.

## Actual tests

-94 XR unit tests plus4 unchanged profile tests passed (98 total), including17
 new geometry/node tests: ray retracing,
 inner-sphere first-entry, tangent-cone shadow,250 randomized shell checks,
 speed bounds, a micrometre-scale tangent-polynomial case,0.5m+ upward input,
 warmup holding, fault stop and fixed resume.
-Real C++ Cyclo precision controller + SG2 PhysX + synthetic WebXR input,
 isolated domain232: all12 integration checks passed,1678 recorded samples.
 No direct fixture JointTrajectory commands and no Quest wearer in this test.
-10cm straight out/back, fixed wrist orientation: raw and published cross-track
 distance0 at floating-point output precision; measured maximum1.394mm left /
 1.393mm right. Final wrist position error about0.0063mm /0.0059mm in simulation.
-Yet joint posture did not return identically: left q2+6.131deg, q3-6.010deg,
 q5+6.165deg; right mirrored. This distinction is reported, not called fixed.
-Overhead fixture endpoint: measured wrist Z=1.91432m in base_link, reference
 displacement0.82587m from calibration. Both arms stayed ACTIVE; endpoint
 wrist error about0.064mm. These are synthetic endpoint figures, not hardware
 or real Quest accuracy guarantees.
-Misaligned X+A was rejected without new goals; aligned restart succeeded;
 one calibration signature persisted; Y+B stopped. Old mode hashes unchanged.
-Intermittent `QP solver failed to converge` warnings still occurred in the
 unchanged controller. It recovered and the measured checks passed. This work
 does not eliminate solver failures or guarantee overhead reach at every pose.

After the small-coefficient tangent fix and its regression test, the complete
real-Cyclo/PhysX sequence was repeated with the final code: all12 checks passed
again. Final measured max cross-track error1.318mm left /1.321mm right;
return endpoint position errors0.0056/0.0058mm; max joint return difference
5.987deg; both overhead wrists Z1.91432m with reference displacement0.82587m
and endpoint tracking error0.064mm. These micro-scale endpoint numbers are
synthetic-simulation readouts, not meaningful hardware accuracy claims.

Evidence:

```text
~/Downloads/.retarget-review/overhead_return_20261014/integration_trial1/
~/Downloads/external_repos/ai_worker_teleop_pc/docker/workspace/extension/overhead_trial1.json
~/Downloads/external_repos/ai_worker/docker/workspace/sg2_arm_logs/extension_extension_trial1/
~/Downloads/.retarget-review/overhead_return_20261014/integration_final/
~/Downloads/external_repos/ai_worker_teleop_pc/docker/workspace/extension/overhead_final.json
~/Downloads/external_repos/ai_worker/docker/workspace/sg2_arm_logs/extension_extension_final/
```

## Commands (only ROBOTIS Applications changes)

Stop the old Vuer process. Use the same Isaac Sim and AI Worker extension
controller as before. In ROBOTIS Applications Docker:

```bash
export ROS_DOMAIN_ID=231
bash /root/ros2_ws/src/robotis_applications/run_sg2_xr_overhead_sim.sh
```

Quest: refresh the same8012 Vuer page and enter passthrough. First X+A captures
once. Wait for XR RESUME READY; WAIT_START still requires matching the visual
markers and a fresh released-then-pressed X+A. No calibration reset on resume.

Logger in a separate AI Worker Docker shell (the installed logger is updated):

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=231 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
LOG_DIR="/workspace/sg2_arm_logs/overhead_$(date +%Y%m%d_%H%M%S)"
python3 -u /workspace/sg2_diagnostics/log_sg2_arm.py \
  --urdf /workspace/sg2_xr_profiles/extension/sg2_near_extension.urdf \
  --output "$LOG_DIR" --hz 10
```

Start this BEFORE the first X+A and do not reuse an existing output directory.
`/sg2_xr/path_diagnostics` adds input/raw/workspace/published/actual matrices
and source/receipt age in events, with matching CSV xyz/quaternion columns.
The other joint, geometric elbow bend and phase/session columns remain.
This is sampled diagnostic data, not hardware-synchronised measurement: check
`path_goal_skew_s` before comparing a path frame with the other CSV sources.
In the integration recording724 fresh path rows were saved.

Rollback: stop overhead Vuer, restart the controller session as normally needed,
and use `run_sg2_xr_guided_sim.sh`; its implementation was not changed.
