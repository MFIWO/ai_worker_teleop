"""Build/select the same Dynamixel driver with HX5 packet writes at 30 Hz.

No ROS node or serial port is opened by this builder. Original installed sources
are preserved. Only model 260/261 writes are paced in the existing write loop.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

SOURCE=Path('/root/ros2_ws/src/dynamixel_hardware_interface')
ROOT=Path('/workspace/ai_worker_xr_hand_write_30hz')
CPP=Path('src/dynamixel/dynamixel.cpp')
HEADER=Path('include/dynamixel_hardware_interface/dynamixel/dynamixel.hpp')
HELPER=Path('include/dynamixel_hardware_interface/dynamixel/hx5_write_cadence.hpp')
TEMPLATE=Path(__file__).with_name('hx5_write_cadence.hpp')
PACKAGE='dynamixel_hardware_interface'
ENV_HZ='AI_WORKER_HX5_WRITE_HZ'
SOURCE_HASHES={str(CPP):'15f36f4df5d93a2246d8fcc3e1b98c2c1d5caa2114441da32b27913ac8023adb',
               str(HEADER):'1d7c7e44e538742ba2603094be2a86b77bf97152be139102e8123422f975752a'}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('Unknown Dynamixel source anchor: '+old[:100])
    return text.replace(old,new,1)


def patch_header(text):
    text=replace_once(text,'#include <map>',
        '#include "dynamixel_hardware_interface/dynamixel/hx5_write_cadence.hpp"\n\n#include <map>')
    return replace_once(text,'  bool write_type_;','  HX5WriteCadence hx5_write_cadence_;\n  bool write_type_;')


def patch_source(text):
    text=replace_once(text,'#include <chrono>','#include <chrono>\n#include <cstdlib>')
    text=replace_once(text,'  dxl_info_.SetDxlModelFolderPath(path);','''  if (const char * rate = std::getenv("AI_WORKER_HX5_WRITE_HZ")) {
    char * end = nullptr;
    const double hz = std::strtod(rate, &end);
    if (end == rate || *end != '\\0') {
      throw std::invalid_argument("Invalid AI_WORKER_HX5_WRITE_HZ");
    }
    hx5_write_cadence_ = HX5WriteCadence(hz);
  }
  dxl_info_.SetDxlModelFolderPath(path);''')
    text=replace_once(text,'  // First, read the model file to get the control table structure','''  if (hx5_write_cadence_.identify(comm_id, id, dxl_model_number)) {
    fprintf(stderr, "[HX5 WRITE] ID%u model%u: latest goals at %s Hz; original read loop retained\\n",
      static_cast<unsigned>(comm_id), static_cast<unsigned>(dxl_model_number),
      std::getenv("AI_WORKER_HX5_WRITE_HZ"));
  }

  // First, read the model file to get the control table structure''')
    for name,group,error in [('Sync','group_sync_write_','SYNC_WRITE_FAIL'),('Bulk','group_bulk_write_','BULK_WRITE_FAIL')]:
        signature=f'DxlError Dynamixel::SetDxlValueTo{name}Write()\n{{'
        begin=text.index(signature)
        end=text.index('\n}\n',begin)+3
        body=text[begin:end]
        body=replace_once(body,signature,signature+'''
  const auto write_now = HX5WriteCadence::now_ns();
  std::vector<uint8_t> packet_ids;''')
        body=replace_once(body,'    uint8_t comm_id = it_write_data.comm_id;',
            '    uint8_t comm_id = it_write_data.comm_id;\n'
            '    if (!hx5_write_cadence_.due(comm_id, write_now)) {continue;}\n'
            '    packet_ids.push_back(comm_id);')
        body=replace_once(body,f'  int dxl_comm_result = {group}->txPacket();',
            '  if (packet_ids.empty()) {return DxlError::OK;}\n'
            f'  int dxl_comm_result = {group}->txPacket();')
        body=replace_once(body,'  } else {\n    return DxlError::OK;\n  }',
            '  } else {\n'
            '    const auto sent_now = HX5WriteCadence::now_ns();\n'
            '    for (const auto id : packet_ids) {hx5_write_cadence_.sent(id, sent_now);}\n'
            '    return DxlError::OK;\n  }')
        text=text[:begin]+body+text[end:]
    return text


def source_hashes(source):
    return {str(p.relative_to(source)):sha(p) for p in sorted(source.rglob('*'))
            if p.is_file() and '.git' not in p.parts and '__pycache__' not in p.parts}


def library(root):return root/'install'/PACKAGE/'lib'/('lib'+PACKAGE+'.so')


def verify(root=ROOT,source=SOURCE):
    root=Path(root);source=Path(source)
    m=json.loads((root/'hand_write_manifest.json').read_text())
    if m.get('purpose')!='hx5_packet_write_cadence' or m.get('hand_write_hz')!=30.0:
        raise ValueError('Unknown HX5 cadence build')
    if source_hashes(source)!=m['original_source_sha256']:
        raise ValueError('Original driver changed; rebuild the hand cadence overlay')
    dest=root/'src'/PACKAGE
    expected={str(CPP):hashlib.sha256(patch_source((source/CPP).read_text()).encode()).hexdigest(),
              str(HEADER):hashlib.sha256(patch_header((source/HEADER).read_text()).encode()).hexdigest(),
              str(HELPER):sha(TEMPLATE)}
    if m['patched_sha256']!=expected or any(sha(dest/p)!=v for p,v in expected.items()):
        raise ValueError('Hand cadence patch changed; rebuild before operation')
    if sha(library(root))!=m['library_sha256']:raise ValueError('Hand cadence library changed')
    return m


def follower_environment(root=ROOT,base=None):
    verify(root)
    env=dict(os.environ if base is None else base)
    prefix=str(Path(root)/'install'/PACKAGE)
    for name,path in [('AMENT_PREFIX_PATH',prefix),('CMAKE_PREFIX_PATH',prefix),('LD_LIBRARY_PATH',prefix+'/lib')]:
        env[name]=path+(':'+env[name] if env.get(name) else '')
    env[ENV_HZ]='30'
    return env


def verify_running_follower(root=ROOT):
    verify(root)
    followers=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            args=(proc/'cmdline').read_bytes().split(b'\0')
            if not args or Path(args[0].decode()).name!='ros2_control_node':continue
            env=(proc/'environ').read_bytes().split(b'\0')
            maps=(proc/'maps').read_text()
        except (OSError,UnicodeDecodeError):continue
        followers.append(ENV_HZ.encode()+b'=30' in env and str(library(Path(root))) in maps)
    if followers!=[True]:raise RuntimeError('Restart the robot session to select the HX5 30 Hz hardware driver')


def build(source=SOURCE,root=ROOT):
    source=Path(source).resolve();root=Path(root).resolve()
    if root==source or root in source.parents or source in root.parents:
        raise ValueError('Build must be separate from the original source')
    if (root/'hand_write_manifest.json').exists():verify(root,source);return
    for name,digest in SOURCE_HASHES.items():
        if sha(source/name)!=digest:raise ValueError('Installed driver version differs; review before building')
    if root.exists() and any(root.iterdir()):raise ValueError('Unfinished build exists; inspect it or use another root')
    original=source_hashes(source);dest=root/'src'/PACKAGE
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copytree(source,dest,ignore=shutil.ignore_patterns('.git','__pycache__'))
    (dest/CPP).write_text(patch_source((source/CPP).read_text()))
    (dest/HEADER).write_text(patch_header((source/HEADER).read_text()))
    shutil.copyfile(TEMPLATE,dest/HELPER)
    subprocess.run(['colcon','build','--packages-select',PACKAGE,'--executor','sequential',
                    '--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF'],
                   cwd=root,env={**os.environ,'CMAKE_BUILD_PARALLEL_LEVEL':'2'},check=True,timeout=900)
    if source_hashes(source)!=original:raise RuntimeError('Original driver changed during build')
    m={'purpose':'hx5_packet_write_cadence','hand_write_hz':30.0,'read_loop_changed':False,
       'arm_writes_changed':False,'robot_commands_sent':False,'original_source_sha256':original,
       'patched_sha256':{str(p):sha(dest/p) for p in (CPP,HEADER,HELPER)},
       'library_sha256':sha(library(root))}
    (root/'hand_write_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    verify(root,source)
    print('[HX5 WRITE BUILD] Original driver path, hand packets=30 Hz; no robot started')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    if args.verify_only:verify();print('[HX5 WRITE BUILD] Verified')
    else:build()


if __name__=='__main__':main()
