"""EXPERIMENTAL HX5 bypass; mixed-bus validation failed. Never starts ROS.

Do not select this build for operation. The existing launcher retains its
previous driver until a bypass passes the physical mixed-bus workload.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from hardware.build_hand_write_rate import SOURCE, CPP, HEADER, SOURCE_HASHES, sha, source_hashes, library, replace_once

ROOT = Path('/workspace/ai_worker_xr_hand_direct')
PACKAGE = 'dynamixel_hardware_interface'
HELPER = Path('include/dynamixel_hardware_interface/dynamixel/hx5_direct_transport.inc')
TEMPLATE = Path(__file__).with_name('hx5_direct_transport.inc')
HARDWARE = Path('src/dynamixel_hardware_interface.cpp')
ENV = 'AI_WORKER_HX5_DIRECT'


def patch_header(text):
    text = replace_once(text, '#include <map>', '#include <map>\n#include <set>\n#include <cmath>')
    return replace_once(text, '  bool write_type_;', '''  bool hx5_direct_enabled_ = false;
  bool hx5_direct_primed_ = false;
  std::set<uint8_t> hx5_direct_boards_;
  std::vector<RWItemList> hx5_direct_reads_, hx5_direct_sensors_, hx5_direct_writes_;
  size_t hx5_direct_read_cursor_ = 0, hx5_direct_sensor_cursor_ = 0, hx5_direct_write_cursor_ = 0;
  bool HX5DirectVirtual(uint8_t comm, uint8_t id) const;
  DxlError AddHX5Direct(uint8_t comm, uint8_t id, const std::vector<std::string> & names,
    const std::vector<std::shared_ptr<double>> & pointers, bool writing);
  DxlError ReadHX5DirectRow(const RWItemList & row);
  DxlError ReadHX5Direct(bool prime);
  DxlError WriteHX5Direct();
  bool write_type_;''')


def patch_source(text):
    text = replace_once(text, '#include <chrono>', '#include <chrono>\n#include <cstdlib>')
    text = replace_once(text, '  dxl_info_.SetDxlModelFolderPath(path);', '''  const char * direct = std::getenv("AI_WORKER_HX5_DIRECT");
  hx5_direct_enabled_ = direct && std::string(direct) == "1";
  dxl_info_.SetDxlModelFolderPath(path);''')
    text = replace_once(text, '  // First, read the model file to get the control table structure', '''  if (hx5_direct_enabled_ && comm_id == id && (dxl_model_number == 260 || dxl_model_number == 261)) {
    hx5_direct_boards_.insert(comm_id);
    fprintf(stderr, "[HX5 DIRECT] Board %u: TableSync bypass; unicast finger I/O on existing port\\n", comm_id);
  }
  // First, read the model file to get the control table structure''')
    for method, flag in [('Read', 'false'), ('Write', 'true')]:
        anchor = f'''DxlError Dynamixel::SetDxl{method}Items(
  uint8_t comm_id,
  uint8_t id,
  std::vector<std::string> item_names,
  std::vector<std::shared_ptr<double>> data_vec_ptr)
{{'''
        text = replace_once(text, anchor, anchor+f'''
  if (HX5DirectVirtual(comm_id, id)) {{
    return AddHX5Direct(comm_id, id, item_names, data_vec_ptr, {flag});
  }}''')
    anchor = 'DxlError Dynamixel::WriteItem(uint8_t comm_id, uint8_t id, std::string item_name, uint32_t data)\n{'
    text = replace_once(text, anchor, anchor+'''
  if (hx5_direct_enabled_ && hx5_direct_boards_.count(comm_id) &&
    comm_id == id && item_name == "Table Sync Enable") {data = 0;}''')
    # Keep all original arm handlers, conversion, limits and packet code.
    anchor = 'DxlError Dynamixel::ReadMultiDxlData(double period_ms)\n{'
    text = replace_once(text, anchor, anchor+'''
  if (hx5_direct_enabled_) {
    const auto result = read_data_list_.empty() ? DxlError::OK :
      (read_type_ == SYNC ? GetDxlValueFromSyncRead(period_ms) : GetDxlValueFromBulkRead(period_ms));
    if (result != DxlError::OK) {return result;}
    return ReadHX5Direct(!hx5_direct_primed_);
  }''')
    anchor = 'DxlError Dynamixel::WriteMultiDxlData()\n{'
    text = replace_once(text, anchor, anchor+'''
  if (hx5_direct_enabled_) {
    const auto result = write_data_list_.empty() ? DxlError::OK :
      (write_type_ == SYNC ? SetDxlValueToSyncWrite() : SetDxlValueToBulkWrite());
    if (result != DxlError::OK) {return result;}
    return WriteHX5Direct();
  }''')
    text = replace_once(text, '}  // namespace dynamixel_hardware_interface',
        '#include "dynamixel_hardware_interface/dynamixel/hx5_direct_transport.inc"\n\n}  // namespace dynamixel_hardware_interface')
    return text


def expected(source):
    return {str(CPP): hashlib.sha256(patch_source((source/CPP).read_text()).encode()).hexdigest(),
            str(HEADER): hashlib.sha256(patch_header((source/HEADER).read_text()).encode()).hexdigest(),
            str(HELPER): sha(TEMPLATE),
            str(HARDWARE): hashlib.sha256(patch_hardware((source/HARDWARE).read_text()).encode()).hexdigest()}


def patch_hardware(text):
    # The upstream runtime ignored WriteMultiDxlData's return value.
    return replace_once(text, '    dxl_comm_->WriteMultiDxlData();', '''    if (dxl_comm_->WriteMultiDxlData() != DxlError::OK) {
      RCLCPP_ERROR_STREAM(logger_, "HX5 direct/arm packet write failed");
      return hardware_interface::return_type::ERROR;
    }''')


def verify(root=ROOT, source=SOURCE):
    root, source = Path(root), Path(source)
    m = json.loads((root/'hand_direct_manifest.json').read_text())
    if m.get('purpose') != 'hx5_direct_unicast' or m.get('read_batch') != 4 or m.get('write_batch') != 8:
        raise ValueError('Unknown HX5 direct transport build')
    if source_hashes(source) != m['original_source_sha256']:
        raise ValueError('Original driver changed; rebuild direct transport')
    dest = root/'src'/PACKAGE
    hashes = expected(source)
    if m['patched_sha256'] != hashes or any(sha(dest/p) != v for p, v in hashes.items()):
        raise ValueError('Direct transport patch changed; rebuild')
    if sha(library(root)) != m['library_sha256']:
        raise ValueError('Direct transport library changed')
    return m


def follower_environment(root=ROOT, base=None):
    verify(root)
    raise RuntimeError('HX5 bypass failed mixed-bus hardware validation; operational selection is disabled')


def verify_running_follower(root=ROOT):
    verify(root)
    raise RuntimeError('HX5 bypass is experimental and has not qualified for operation')


def build(source=SOURCE, root=ROOT):
    source, root = Path(source).resolve(), Path(root).resolve()
    if root == source or root in source.parents or source in root.parents:
        raise ValueError('Build must be separate from original driver')
    if (root/'hand_direct_manifest.json').exists():verify(root, source);return
    if root.exists() and any(root.iterdir()):raise ValueError('Inspect unfinished direct build or choose a new root')
    for name, digest in SOURCE_HASHES.items():
        if sha(source/name) != digest:raise ValueError('Installed driver differs from reviewed source')
    original = source_hashes(source)
    dest = root/'src'/PACKAGE
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, dest, ignore=shutil.ignore_patterns('.git', '__pycache__'))
    (dest/CPP).write_text(patch_source((source/CPP).read_text()))
    (dest/HEADER).write_text(patch_header((source/HEADER).read_text()))
    (dest/HARDWARE).write_text(patch_hardware((source/HARDWARE).read_text()))
    shutil.copyfile(TEMPLATE, dest/HELPER)
    subprocess.run(['colcon', 'build', '--packages-select', PACKAGE, '--executor', 'sequential',
                    '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'],
                   cwd=root, env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': '2'}, check=True, timeout=900)
    if source_hashes(source) != original:raise RuntimeError('Original driver changed during build')
    m = dict(purpose='hx5_direct_unicast', read_batch=4, write_batch=8,
             original_source_sha256=original, patched_sha256=expected(source),
             library_sha256=sha(library(root)), robot_commands_sent=False)
    (root/'hand_direct_manifest.json').write_text(json.dumps(m, indent=2)+'\n')
    verify(root, source)
    print('[HX5 DIRECT BUILD] Verified; no ROS process or motor started')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    if args.verify_only:verify(args.root)
    else:build(root=args.root)


if __name__ == '__main__':main()
