"""Check safe installation and real compiled UI routing without starting Loop."""
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('tactile_patch', Path(__file__).with_name('patch_tactile_viewer.py'))
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)
INSTALLED = Path('/opt/Loop/_internal/frontend/dist') / patch.ASSET


class TactileViewerPatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not INSTALLED.exists():
            raise unittest.SkipTest('Loop Desktop 1.2.1 bundle unavailable')
        backup = INSTALLED.with_name(INSTALLED.name + patch.BACKUP_SUFFIX)
        cls.original = backup.read_bytes() if backup.exists() else INSTALLED.read_bytes()

    def test_reversible_idempotent_install_and_unknown_version_rejected(self):
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            target = Path(folder) / patch.ASSET
            target.parent.mkdir(parents=True)
            target.write_bytes(self.original)
            patch.apply(folder)
            first = target.read_bytes()
            patch.apply(folder)
            self.assertEqual(first, target.read_bytes())
            patch.restore(folder)
            self.assertEqual(target.read_bytes(), self.original)
            target.write_bytes(b'unknown Loop update')
            with self.assertRaises(ValueError):
                patch.apply(folder)
            self.assertEqual(target.read_bytes(), b'unknown Loop update')

    @unittest.skipUnless(shutil.which('node'), 'Node needed for compiled UI test')
    def test_numeric_tactile_uses_chart_and_seek_but_force_map_is_preserved(self):
        text = patch.patched_bytes(self.original).decode()
        def function(name):
            start = text.index('function ' + name + '(')
            return text[start:text.index('function ', start + 10)]
        script = r'''
const assert = require('node:assert/strict');
const jsx = (type, props, key) => ({type, props, key});
const L = {jsx, jsxs:jsx};
const I = {useState:()=>[[],()=>{}],useEffect:()=>{},useMemo:f=>f()};
const P='card',O='header',N='title',A='badge',M='body',Y='warnings',ar='chart',or='fields';
const Tr='force-map',fr='sample-only';
const rn = slice => !!slice.forceMap;
const seek=()=>{};
''' + function('ir') + function('rr') + r'''
const robot={sourceId:'robot',kind:'robot',series:[]};
const tactile={sourceId:'hx5',label:'HX5-D20',kind:'tactile',series:[{key:'right_hand[0]',values:[0,255,0]}],warnings:[]};
const force={sourceId:'force',kind:'tactile',forceMap:true};
const other={sourceId:'other',kind:'marker'};
const sources=[robot,tactile,force,other];
const output=rr({sources,currentTimeUs:8670000,onSeek:seek,query:{data:{slices:sources,startTimeUs:0,endTimeUs:28410000}}});
const cards=output.props.children[2];
assert.equal(cards[0].type,ir);
assert.equal(cards[1].type,ir);
assert.equal(cards[2].type,Tr);
assert.equal(cards[3].type,fr);
assert.equal(cards[1].props.sourceSlice,tactile);
assert.equal(cards[1].props.onSeek,seek);
assert.equal(cards[1].props.currentTimeUs,8670000);
const rendered=ir(cards[1].props);
assert.ok(JSON.stringify(rendered).includes('Tactile'));
assert.ok(JSON.stringify(rendered).includes('fields'));
assert.ok(JSON.stringify(rendered).includes('chart'));
assert.equal(tactile.kind,'tactile');
'''
        subprocess.run(['node', '-e', script], check=True, capture_output=True, text=True)
        with tempfile.TemporaryDirectory() as folder:
            module = Path(folder) / 'patched.mjs'
            module.write_text(text)
            subprocess.run(['node', '--check', str(module)], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
