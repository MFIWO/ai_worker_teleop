"""Build optional timing-only overlay atop the existing 30Hz hand driver.

No ROS nodes, serial ports, motor goals or setting writes. Selection is explicit.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
from hardware import build_hand_write_rate as base

ROOT=Path('/workspace/ai_worker_xr_hand_trace')
PACKAGE=base.PACKAGE
HARDWARE=Path('src/dynamixel_hardware_interface.cpp')
INCLUDE=Path('include/dynamixel_hardware_interface/dynamixel')
HELPERS=('hx5_stage_trace.hpp','hx5_trace_port.hpp','hx5_write_cadence.hpp')
ENV='AI_WORKER_HX5_TRACE_DIR'


def patch_header(text):
    text=base.patch_header(text)
    return base.replace_once(text,'  bool write_type_;',
                            '  uint64_t hx5_trace_read_cycle_ = 0, hx5_trace_write_cycle_ = 0;\n  bool write_type_;')


def patch_source(text):
    text=base.patch_source(text)
    text=base.replace_once(text,'#include <chrono>',
        '#include "dynamixel_hardware_interface/dynamixel/hx5_trace_port.hpp"\n#include <chrono>')
    text=base.replace_once(text,'  port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());',
        '''  hx5trace::recorder().source(this, port_name);
  port_handler_ = std::getenv("AI_WORKER_HX5_TRACE_DIR") ?
    new HX5TracePort(port_name.c_str(), this) :
    dynamixel::PortHandler::getPortHandler(port_name.c_str());''')
    for method,begin,end in [('Read','DRIVER_READ_ENTER','DRIVER_READ_EXIT'),
                             ('Write','DRIVER_WRITE_ENTER','DRIVER_WRITE_EXIT')]:
        signature=(f'DxlError Dynamixel::{method}MultiDxlData(double period_ms)\n{{' if method=='Read'
                   else 'DxlError Dynamixel::WriteMultiDxlData()\n{')
        text=base.replace_once(text,signature,signature+f'''
  hx5trace::Scope trace_scope(this, hx5trace::{begin}, hx5trace::{end}, ++hx5_trace_{method.lower()}_cycle_);''')
    # Raw is the SDK getData value AFTER packet parsing; wire bytes are recorded separately.
    anchor='    uint32_t dxl_getdata = get_data_func(comm_id, current_addr, size);'
    begin=text.index('DxlError Dynamixel::ProcessReadData(')
    end=text.index('DxlError Dynamixel::ProcessDirectReadData(',begin)
    block=text[begin:end]
    block=base.replace_once(block,anchor,anchor+'''
    if (hx5trace::hand(comm_id) && item_names[item_index] == "Present Position") {
      hx5trace::recorder().emit(hx5trace::RAW_POSITION, this, comm_id, ID, current_addr, 0, dxl_getdata);
    }''')
    anchor='''    }
  }
  return DxlError::OK;
}'''
    block=base.replace_once(block,anchor,'''    }
    if (hx5trace::hand(comm_id) && item_names[item_index] == "Present Position") {
      hx5trace::recorder().emit(hx5trace::CONVERTED_POSITION, this, comm_id, ID, current_addr, 0, *data_ptrs[item_index]);
    }
  }
  return DxlError::OK;
}''')
    text=text[:begin]+block+text[end:]
    for method in ('Sync','Bulk'):
        begin=text.index(f'DxlError Dynamixel::SetDxlValueTo{method}Write()\n{{')
        end=text.index('\n}\n',begin)+3
        block=text[begin:end]
        anchor='      added_byte += size;'
        hook='''      if (hx5trace::hand(comm_id) && item_name == "Goal Position") {
        hx5trace::recorder().emit(hx5trace::GOAL_VALUE, this, comm_id, ID, added_byte, 0, data);
        hx5trace::recorder().emit(hx5trace::PACKED_GOAL, this, comm_id, ID, added_byte, 0, 0, param_write_value+added_byte, size);
      }
'''
        # Bulk has direct and indirect blocks, with distinct indentation.
        if method=='Sync':
            block=base.replace_once(block,anchor,hook+anchor)
        else:
            anchor='        added_byte += size;'
            if block.count(anchor)!=2:raise ValueError('Unknown Bulk write branches')
            block=block.replace(anchor,hook+anchor)
        text=text[:begin]+block+text[end:]
    return text


def patch_hardware(text):
    text='#include "dynamixel_hardware_interface/dynamixel/hx5_stage_trace.hpp"\n'+text
    # Hardware identity distinguishes follower, base and sensor warnings.
    anchor='''  std::vector<uint8_t> reboot_dxl;'''
    text=base.replace_once(text,anchor,'''  hx5trace::recorder().source(dxl_comm_.get(), info_.name);

'''+anchor)
    for method,kind in [('read','HW_READ_ENTER'),('write','HW_WRITE_ENTER')]:
        anchor=f'''hardware_interface::return_type DynamixelHardware::{method}(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{{'''
        text=base.replace_once(text,anchor,anchor+f'\n  hx5trace::recorder().emit(hx5trace::{kind}, dxl_comm_.get());')
    anchor='    dxl_comm_err_ = CheckError(dxl_comm_->ReadMultiDxlData(period_ms));'
    text=base.replace_once(text,anchor,anchor+'''
    hx5trace::recorder().emit(hx5trace::HW_READ_RESULT, dxl_comm_.get(), 0, 0, 0, static_cast<int>(dxl_comm_err_));''')
    # Only the cyclic read, not initialization.
    begin=text.index('hardware_interface::return_type DynamixelHardware::read(')
    end=text.index('hardware_interface::return_type DynamixelHardware::write(',begin)
    block=text[begin:end]
    block=base.replace_once(block,'  CalcTransmissionToJoint();',
        '  CalcTransmissionToJoint();\n  hx5trace::joint_values(hx5trace::HW_STATE, dxl_comm_.get(), hdl_joint_states_);')
    text=text[:begin]+block+text[end:]
    begin=text.index('hardware_interface::return_type DynamixelHardware::write(')
    end=text.index('\n}\n',begin)+3
    block=text[begin:end]
    block=base.replace_once(block,'    CalcJointToTransmission();',
        '    hx5trace::joint_values(hx5trace::HW_GOAL, dxl_comm_.get(), hdl_joint_commands_);\n    CalcJointToTransmission();')
    # Preserve the existing return-value handling; observe without changing behavior.
    block=base.replace_once(block,'    dxl_comm_->WriteMultiDxlData();',
        '    const auto observed_write_result = dxl_comm_->WriteMultiDxlData();\n'
        '    hx5trace::recorder().emit(hx5trace::HW_WRITE_EXIT, dxl_comm_.get(), 0, 0, 0, static_cast<int>(observed_write_result));')
    return text[:begin]+block+text[end:]


def expected(source):
    out={str(base.CPP):patch_source((source/base.CPP).read_text()),
         str(base.HEADER):patch_header((source/base.HEADER).read_text()),
         str(HARDWARE):patch_hardware((source/HARDWARE).read_text())}
    for name in HELPERS:
        out[str(INCLUDE/name)]=Path(__file__).with_name(name).read_text()
    return out


def verify(root=ROOT,source=base.SOURCE):
    root,source=Path(root),Path(source)
    manifest=json.loads((root/'trace_manifest.json').read_text())
    if manifest.get('purpose')!='hx5_stage_timing_only_v1':raise ValueError('Unknown trace build')
    if base.source_hashes(source)!=manifest['original_source_sha256']:raise ValueError('Original driver changed')
    for name,text in expected(source).items():
        digest=hashlib.sha256(text.encode()).hexdigest()
        if manifest['patched_sha256'].get(name)!=digest or base.sha(root/'src'/PACKAGE/name)!=digest:
            raise ValueError('Trace source changed: '+name)
    if base.sha(base.library(root))!=manifest['library_sha256']:raise ValueError('Trace library changed')
    return manifest


def follower_environment(trace_dir,root=ROOT,base_env=None):
    verify(root)
    dest=Path(trace_dir)
    if not dest.is_absolute():raise ValueError('Trace directory must be absolute')
    dest.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ if base_env is None else base_env)
    prefix=str(Path(root)/'install'/PACKAGE)
    for name,path in [('AMENT_PREFIX_PATH',prefix),('CMAKE_PREFIX_PATH',prefix),('LD_LIBRARY_PATH',prefix+'/lib')]:
        env[name]=path+(':'+env[name] if env.get(name) else '')
    env['AI_WORKER_HX5_WRITE_HZ']='30'
    env[ENV]=str(dest)
    return env


def verify_running_follower(root=ROOT):
    verify(root);found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            args=(p/'cmdline').read_bytes().split(b'\0')
            if not args or Path(args[0].decode()).name!='ros2_control_node':continue
            env=(p/'environ').read_bytes().split(b'\0');maps=(p/'maps').read_text()
            found.append(any(s.startswith(ENV.encode()+b'=') for s in env) and str(base.library(Path(root))) in maps)
        except (OSError,UnicodeError):continue
    if found!=[True]:raise RuntimeError('Restart robot session with --hand-trace --bringup; existing follower is not the timing build')


def build(source=base.SOURCE,root=ROOT):
    source,root=Path(source).resolve(),Path(root).resolve()
    if root==source or root in source.parents or source in root.parents:raise ValueError('Separate build required')
    if (root/'trace_manifest.json').exists():verify(root,source);return
    for name,digest in base.SOURCE_HASHES.items():
        if base.sha(source/name)!=digest:raise ValueError('Unexpected installed driver version')
    if root.exists() and any(root.iterdir()):raise ValueError('Unfinished trace build; inspect before retrying')
    original=base.source_hashes(source);dest=root/'src'/PACKAGE
    shutil.copytree(source,dest,ignore=shutil.ignore_patterns('.git','__pycache__'))
    patched=expected(source)
    for name,text in patched.items():(dest/name).write_text(text)
    subprocess.run(['nice','-n','19','colcon','build','--packages-select',PACKAGE,'--executor','sequential',
                    '--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF'],cwd=root,
                   env={**os.environ,'CMAKE_BUILD_PARALLEL_LEVEL':'1'},check=True,timeout=900)
    if base.source_hashes(source)!=original:raise RuntimeError('Original driver changed during build')
    manifest={'purpose':'hx5_stage_timing_only_v1','original_source_sha256':original,
              'patched_sha256':{name:base.sha(dest/name) for name in patched},
              'library_sha256':base.sha(base.library(root)),'motor_commands_sent':False,
              'control_behavior_changed':False,'hand_write_hz':30.}
    (root/'trace_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');verify(root,source)
    print('[HAND TRACE BUILD] Timing-only overlay ready; no robot started')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    if args.verify_only:verify();print('[HAND TRACE BUILD] Verified')
    else:build()

if __name__=='__main__':main()
