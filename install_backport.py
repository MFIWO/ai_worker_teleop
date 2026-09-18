"""Apply the reviewed SG2 backport without overwriting unrelated changes."""

from pathlib import Path
import shutil


bundle = Path(__file__).resolve().parent
source = Path('/root/ros2_ws/src/cyclo_control')
updates = sorted(path for path in (bundle / 'files').rglob('*') if path.is_file()
                 and '__pycache__' not in path.parts)

# Validate every target before modifying any source file.
for update in updates:
    relative = update.relative_to(bundle / 'files')
    target = source / relative
    original = bundle / 'original' / relative
    if target.exists() and target.read_bytes() == update.read_bytes():
        continue
    if original.exists():
        if not target.exists() or target.read_bytes() != original.read_bytes():
            raise SystemExit(f'Unrelated change detected; not overwriting {target}')
    elif target.exists():
        raise SystemExit(f'New-file collision; not overwriting {target}')

for update in updates:
    target = source / update.relative_to(bundle / 'files')
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(update, target)
    print(f'Installed {target}')

print('Sources installed. Build cyclo_motion_controller_ros_py and '
      'cyclo_motion_controller_ros before restarting VR.')
