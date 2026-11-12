#!/usr/bin/env python3
"""Check that archived source bytes still match the captured baseline."""
from pathlib import Path
import hashlib,json,sys
root=Path(__file__).resolve().parents[3]
manifest=json.loads((root/'airsim/hw-ros2/baseline/source_manifest.json').read_text())
errors=[]
for item in manifest['files']:
 p=root/item['file']
 if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=item['sha256']:errors.append(item['file'])
if errors:
 print('MISMATCH:',*errors,sep='\n');sys.exit(1)
print('PASS:',len(manifest['files']),'archived source/config/license files match baseline SHA-256.')
