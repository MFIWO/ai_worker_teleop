# HX5 driver timing capture

This optional diagnostic keeps mode 1/2, the existing 30Hz hand write cadence,
arm control, packet selection, motor goals, profiles and SDK settings. It wraps
SDK readPort/writePort calls and observes driver values; it never introduces a
serial read/write or retry. It must be selected before starting the follower.
The default robot command continues using its previous driver.

## Operator commands (all from PC host terminals)

The 192.0.2.10 PC and 192.0.2.20 robot addresses below are documentation
examples. Replace both with your actual addresses.

Close old PC/robot teleop sessions first. The usual robot bringup retains its
five-second arms-only ready movement. Open Loop as for normal `--loop vr` use.

Robot session:

```bash
bash ~/Downloads/external_repos/ai_worker_teleop_deploy/teleop.sh robot \
  --pc-ip 192.0.2.10 --robot-ip 192.0.2.20 \
  --loop vr --input hands --hand-mode 1 --bringup --hand-trace
```

PC session:

```bash
bash ~/Downloads/external_repos/ai_worker_teleop_deploy/teleop.sh pc \
  --pc-ip 192.0.2.10 --robot-ip 192.0.2.20 \
  --loop vr --input hands --hand-mode 1
```

Once teleop is ready, third PC terminal:

```bash
ssh -t robotis@192.0.2.20 \
  'docker exec -it ai_worker bash /workspace/ai_worker_xr/hardware/log_hands.sh --driver-trace --seconds 120'
```

Begin the hand experiment when the wrapper prints `[HAND LOG RECORDING]`.
The wrapper waits for the ROS recorder to become ready before arming driver
observation, and disables recorder stdin keyboard controls. This avoids the
interactive TTY/process-group stop reproduced with the earlier timeout wrapper. The recorder
stops itself after 120 seconds; Ctrl+C in this third terminal also stops only
recording. It does not stop teleop or turn off torque. Driver capture supports
10..180 seconds per request. No live reader should open the recording database.

Example movements: five seconds still; slow open/close; faster open/close;
left index/middle/ring/little individually; right individually; five seconds
still. Gentle movements are sufficient; no forceful closure is required.

`[HAND LOG SAVED]` identifies the container directory:
`/workspace/ai_worker_xr_logs/hand_mode1_<KST timestamp>_<pid>/`.
The folder contains ROS bag, manifest including loaded SDK/driver paths,
`driver_trace/trace_<token>.bin`, final drop/I/O/performance counters,
`sources.tsv`, and a snapshot of the follower log. The timing recorder remains
inactive until this explicit capture request; logging uses a bounded queue and
background file writer. It records sequence/drop counters rather than blocking
control when the queue or logger lock is busy. Disk use can reach hundreds of MB.

After recording has finalized, offline stage decoding is available in the robot
container (no graph/serial access):

```bash
cd /workspace/ai_worker_xr
python3 -B -m hardware.analyse_hand_trace /workspace/ai_worker_xr_logs/<saved_folder>/driver_trace
```

Normal operation is restored by closing the diagnostic robot session and
starting the usual robot command without `--hand-trace`.

## What is measured

- `HW_GOAL`: position seen by hardware write before joint/transmission mapping.
- `GOAL_VALUE`, `PACKED_GOAL`: selected goal and encoded bytes on the paced row.
- `SERIAL_TX`, `SERIAL_TX_RETURN`: bytes passed to SDK host write and its result.
- `SERIAL_RX`: bytes actually returned by SDK host read, including all devices.
- `RAW_POSITION`: the SDK's decoded getData output, with board/motor/address.
- `CONVERTED_POSITION`: position after driver unit conversion.
- `HW_STATE`: position after transmission/joint mapping.
- Hardware/driver entry/exit times and actual read/write result codes.

Sources are keyed by driver instance and mapped to port and hardware component
names. This prevents sensor/base errors from being silently treated as hand
follower errors. The binary header anchors steady time to wall time for bag
alignment; wall-clock steps must still be checked during analysis.

Host write completion does not establish on-wire completion, board receipt or
motor application. SDK getData can differ from actual reply bytes; CRC-valid
normal protocol2 reply payloads are exported separately. FastRead aggregate
payloads may require additional SDK-specific interpretation. Dropped trace
records invalidate incomplete timing chains. Physical motion timing or internal
board tracing is still needed if board replies themselves contain old positions.

## Validation performed

The isolated overlay compiles on the robot. Original packet-call counts are
preserved by the patch. A fake SDK test confirms exactly one inherited call and
unchanged return values/buffers. Offline selection/capture/CRC tests pass.
On the robot CPU, a paced fake-event test wrote 9,997 of 10,003 observations,
counted six nonblocking lock drops, and measured emit p99 at 416ns. This test
opens no serial ports; it does not prove the real control loop has zero added
latency. The first instrumented trial must also check cycle timing and drops.
No physical teleop was started as part of preparing this diagnostic.
