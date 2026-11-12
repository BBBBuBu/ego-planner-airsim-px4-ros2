#!/usr/bin/env python3
"""Restore pinned PX4 sources while preserving the archived local overlays."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(directory, args, direct=False):
    env = os.environ.copy()
    if direct:
        for name in list(env):
            if name.lower() in {'http_proxy', 'https_proxy', 'all_proxy'}:
                env.pop(name, None)
    prefix = ['git'] + (['-c', 'http.proxy='] if direct else [])
    return subprocess.check_output(prefix + ['-C', str(directory), *args], env=env, text=True).strip()


def prepare(target, direct=False):
    target = Path(target).resolve()
    marker = target / 'BASELINE_SOURCE.json'
    lock = json.loads(marker.read_text())
    overlays = lock['overlay_files']
    for name, expected in overlays.items():
        if not (target/name).is_file() or sha(target/name) != expected:
            raise RuntimeError(f'Local overlay differs from archived snapshot: {name}; no files overwritten.')
    if (target/'.git').exists():
        if git(target, ['rev-parse', 'HEAD'], direct) != lock['commit']:
            raise RuntimeError('Existing PX4 checkout uses a different commit; no reset or overwrite performed.')
        changed = git(target, ['-c', 'core.fileMode=false', 'diff', '--ignore-space-at-eol', '--name-only', 'HEAD'], direct).splitlines()
        if set(changed) - set(overlays):
            raise RuntimeError('Existing PX4 has additional local modifications; no reset or overwrite performed.')
        print('PX4 checkout and local overlays already match the baseline.')
        return
    allowed = {'BASELINE_SOURCE.json', *overlays}
    unexpected = [str(p.relative_to(target)) for p in target.rglob('*') if p.is_file() and str(p.relative_to(target)) not in allowed]
    if unexpected:
        raise RuntimeError('PX4 target contains unrelated files; refusing to overwrite: ' + ', '.join(unexpected[:5]))
    with tempfile.TemporaryDirectory(prefix='.px4-download-', dir=target.parent) as temporary:
        checkout = Path(temporary)/'checkout';checkout.mkdir()
        git(checkout, ['init', '-q'], direct)
        git(checkout, ['remote', 'add', 'origin', lock['repository']], direct)
        git(checkout, ['fetch', '--depth', '1', 'origin', lock['commit']], direct)
        git(checkout, ['checkout', '-q', '--detach', 'FETCH_HEAD'], direct)
        version = lock.get('version')
        if version:
            git(checkout, ['fetch', '--depth', '1', 'origin', f'refs/tags/{version}:refs/tags/{version}'], direct)
            if git(checkout, ['rev-parse', f'{version}^{{commit}}'], direct) != lock['commit']:
                raise RuntimeError('Version tag does not match pinned commit.')
        if lock.get('recursive_submodules', True):
            git(checkout, ['submodule', 'update', '--init', '--recursive', '--depth', '1'], direct)
        if git(checkout, ['rev-parse', 'HEAD'], direct) != lock['commit']:
            raise RuntimeError('Downloaded commit does not match baseline.')
        # Overlay files stay in place; all other source files and Git metadata are restored.
        def merge(source, destination):
            for item in source.iterdir():
                out = destination/item.name
                if out.exists():
                    if item.is_dir() and out.is_dir():
                        merge(item, out)
                    elif str(out.relative_to(target)) not in overlays:
                        raise RuntimeError(f'Unexpected collision: {out.name}')
                else:
                    shutil.move(str(item), str(out))
        merge(checkout, target)
    for name, expected in overlays.items():
        if sha(target/name) != expected:
            raise RuntimeError(f'Overlay verification failed: {name}')
    print('PX4 prepared at', target)
    print('Pinned commit:', lock['commit'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--direct', action='store_true', help='Bypass proxy environment for public Git downloads')
    args = parser.parse_args()
    target = Path(__file__).resolve().parents[1]/'px4v1.15.2'
    try:
        prepare(target, args.direct)
    except (RuntimeError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, str(error)+'\n')


if __name__ == '__main__':
    main()
