#!/usr/bin/env python3
"""Version-checked, reversible numeric Tactile chart patch for Loop Desktop 1.2.1.

Reuses the installed Robot State chart, seeking, and field selector. Keeps the
source kind and recording format Tactile; specialized force-map views stay intact.
Never starts/stops Loop, changes recordings, or changes robot control.
"""
import argparse
import hashlib
from pathlib import Path
import shutil

ASSET = 'assets/src-BmAAgLCU.js'
ORIGINAL_SHA256 = '494d227c3da36f109ee55772834f61f928ebdd0a143de8be40e3e64f58ea2d6c'
BACKUP_SUFFIX = '.before-teleop-tactile-graph'
REPLACEMENTS = (
    ('e.kind===`robot`?(0,L.jsx)(ir,{currentTimeUs:t,onSeek:n,',
     '(e.kind===`robot`||e.kind===`tactile`&&!rn(e))?(0,L.jsx)(ir,{currentTimeUs:t,onSeek:n,'),
    ('children:`Robot`})})]})}),(0,L.jsxs)(M,{className:`space-y-3`,children:[(0,L.jsx)(ar,',
     'children:e.kind===`tactile`?`Tactile`:`Robot`})})]})}),(0,L.jsxs)(M,{className:`space-y-3`,children:[(0,L.jsx)(ar,'),
    ('children:`No numeric Robot series`', 'children:`No numeric series`'),
)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def patched_bytes(original):
    if sha(original) != ORIGINAL_SHA256:
        raise ValueError('Unsupported Loop frontend bundle; refusing to patch another version')
    text = original.decode('utf-8')
    for before, after in REPLACEMENTS:
        if text.count(before) != 1:
            raise ValueError('Unexpected frontend chart layout')
        text = text.replace(before, after, 1)
    return text.encode('utf-8')


def apply(root):
    target = Path(root) / ASSET
    backup = target.with_name(target.name + BACKUP_SUFFIX)
    current = target.read_bytes()
    if backup.exists():
        expected = patched_bytes(backup.read_bytes())
        if current == expected:
            print('[LOOP TACTILE GRAPH] Already installed')
            return
        if sha(current) != ORIGINAL_SHA256:
            raise ValueError('Frontend changed since backup; refusing to overwrite it')
    else:
        expected = patched_bytes(current)
        shutil.copy2(target, backup)
    temporary = target.with_name(target.name + '.teleop-new')
    temporary.write_bytes(expected)
    shutil.copymode(target, temporary)
    temporary.replace(target)
    print('[LOOP TACTILE GRAPH] Installed; source kind and saved data remain Tactile')
    print('[BACKUP]', backup)


def restore(root):
    target = Path(root) / ASSET
    backup = target.with_name(target.name + BACKUP_SUFFIX)
    original = backup.read_bytes()
    expected = patched_bytes(original)
    if target.read_bytes() not in (original, expected):
        raise ValueError('Frontend changed after patch; refusing to overwrite it')
    shutil.copy2(backup, target)
    print('[LOOP TACTILE GRAPH] Original bundle restored')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frontend-root', type=Path, default=Path('/opt/Loop/_internal/frontend/dist'))
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--apply', action='store_true')
    actions.add_argument('--restore', action='store_true')
    actions.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.apply:
        apply(args.frontend_root)
    elif args.restore:
        restore(args.frontend_root)
    else:
        target = args.frontend_root / ASSET
        backup = target.with_name(target.name + BACKUP_SUFFIX)
        current = target.read_bytes()
        if sha(current) == ORIGINAL_SHA256:
            patched_bytes(current)
            print('[LOOP TACTILE GRAPH] Compatible Loop 1.2.1 frontend; patch not installed')
        elif backup.exists() and current == patched_bytes(backup.read_bytes()):
            print('[LOOP TACTILE GRAPH] Patched frontend verified')
        else:
            raise ValueError('Unsupported or modified Loop frontend')


if __name__ == '__main__':
    main()
