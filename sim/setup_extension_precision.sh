#!/usr/bin/env bash
# [ADDED] Optional SIM precision overlay; installed Cyclo sources are never edited.
# Tightens solver convergence tolerance, not collision/joint/safety constraints.
set -eo pipefail
test -f /opt/ros/jazzy/setup.bash || { echo '[ERROR] Run inside ai_worker Docker'; exit 1; }
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
test "${ROS_DOMAIN_ID:-}" = 231 -o "${ROS_DOMAIN_ID:-}" = 232 || { echo '[ERROR] simulation domain required'; exit 1; }
ROOT=/workspace/sg2_extension_overlay
SRC=/root/ros2_ws/src/cyclo_control/cyclo_motion_controller_core
test -d "$SRC" || { echo "[ERROR] missing $SRC"; exit 1; }
if [[ ! -d "$ROOT/src/cyclo_motion_controller_core" ]]; then
  mkdir -p "$ROOT/src"
  cp -a "$SRC" "$ROOT/src/cyclo_motion_controller_core"
fi
python3 - "$SRC" "$ROOT" <<'PY'
from pathlib import Path
import hashlib,json,sys
source=Path(sys.argv[1]);root=Path(sys.argv[2])
relative=Path('include/cyclo_motion_controller_core/optimization/qp_base.hpp')
p=root/'src/cyclo_motion_controller_core'/relative
original=(source/relative).read_bytes()
old='    solver_.settings()->getSettings()->verbose = false;'
new=old+'''
    // [ADDED SG2 SIM OVERLAY] Lower numerical termination error only.
    // The environment opt-in leaves all installed/default solvers unchanged.
    const char * precision = std::getenv("CYCLO_SG2_QP_PRECISION");
    if (precision != nullptr && std::string(precision) == "1") {
      solver_.settings()->setAbsoluteTolerance(1e-5);
      solver_.settings()->setRelativeTolerance(1e-5);
    }'''
text=p.read_text()
expected=original.decode().replace('#include <Eigen/Dense>','#include <Eigen/Dense>\n#include <cstdlib>',1).replace(old,new,1)
if original.decode().count(old)!=1: raise RuntimeError('Unknown QPBase source; refusing automatic patch')
if text not in (original.decode(),expected): raise RuntimeError('Overlay has unrelated modifications')
if text!=expected: p.write_text(expected)
if (source/relative).read_bytes()!=original: raise RuntimeError('Installed source changed')
manifest={'source':str(source/relative),'source_sha256':hashlib.sha256(original).hexdigest(),
          'overlay_sha256':hashlib.sha256(expected.encode()).hexdigest(),
          'absolute_tolerance':1e-5,'relative_tolerance':1e-5,'simulation_only':True,
          'enable_environment':'CYCLO_SG2_QP_PRECISION=1'}
(root/'precision_manifest.json').write_text(json.dumps(manifest,indent=2))
print('[OVERLAY]',json.dumps(manifest))
PY
cd "$ROOT"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-select cyclo_motion_controller_core --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
echo "[OK] Optional precision core installed at $ROOT/install; original sources untouched."
