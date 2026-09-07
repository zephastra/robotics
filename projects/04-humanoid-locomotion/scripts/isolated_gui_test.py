"""Run GUI checks on a private Xvfb display, with no desktop input interference.

Optional developer-only dependency: Xvfb on PATH, or unpacked under
.cache/gui-test-runtime/root. Does not install packages or modify system settings.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',choices=['g1','t800'],default='t800')
    parser.add_argument('--unit-only',action='store_true')
    parser.add_argument('--viewer',action='store_true',help='Also render the MuJoCo viewer in the private display')
    args=parser.parse_args()
    private=ROOT/'.cache/gui-test-runtime/root'
    binary=shutil.which('Xvfb') or str(private/'usr/bin/Xvfb')
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=str(private/'usr/lib/x86_64-linux-gnu')+':'+str(private/'lib/x86_64-linux-gnu')+':'+env.get('LD_LIBRARY_PATH','')
    env.update(H004_TEST_GUI='1',PYTHONPATH=str(ROOT/'src'),PYTHONDONTWRITEBYTECODE='1',GALLIUM_DRIVER='llvmpipe')
    log_dir=ROOT/'.cache/gui-test-runtime'
    log_dir.mkdir(parents=True,exist_ok=True)
    server_cwd=ROOT
    if binary==str(private/'usr/bin/Xvfb'):
        # The unpacked Ubuntu binary hardcodes /usr/bin/xkbcomp. Relocate that
        # prefix ONLY in a private copy of the optional test tool. Original .deb
        # and extracted executable stay intact; no changes to /usr/bin.
        original=Path(binary).read_bytes()
        if original.count(b'/usr/bin\0')!=1:
            raise RuntimeError('Unsupported private Xvfb binary; install a normal developer Xvfb instead')
        patched=original.replace(b'/usr/bin\0',b'./bin\0\0\0\0')
        local=private/'usr/Xvfb-local'
        local.write_bytes(patched)
        local.chmod(0o755)
        (log_dir/'xvfb-relocation.json').write_text(json.dumps(dict(
            original_sha256=hashlib.sha256(original).hexdigest(),
            relocated_sha256=hashlib.sha256(patched).hexdigest(),
            change='/usr/bin -> ./bin in private test copy only'),indent=2)+'\n')
        binary=str(local)
        server_cwd=private/'usr'
    with (log_dir/'xvfb.log').open('w') as log:
        # WSLg owns a read-only /tmp/.X11-unix mount. Use abstract local sockets;
        # never chmod that mount or expose an unauthenticated TCP display.
        server=subprocess.Popen([binary,':193','-displayfd','1','-screen','0','960x720x24','-nolisten','tcp','-nolisten','unix'],
                                stdout=subprocess.PIPE,stderr=log,env=env,text=True,cwd=server_cwd)
        try:
            if not select.select([server.stdout],[],[],10)[0]:
                raise RuntimeError('Xvfb startup timeout; see '+str(log_dir/'xvfb.log'))
            display=server.stdout.readline().strip()
            if not display.isdigit():
                raise RuntimeError('Xvfb failed; see '+str(log_dir/'xvfb.log'))
            env['DISPLAY']=':'+display
            # Tk must use this X server rather than a host Wayland connection.
            env.pop('WAYLAND_DISPLAY',None)
            print('Private GUI test display:',env['DISPLAY'],flush=True)
            if args.unit_only:
                command=[sys.executable,'-m','pytest','tests','-q','-p','no:cacheprovider']
            else:
                command=[sys.executable,'scripts/verify_manual.py','--robot',args.robot]
                if not args.viewer: command+=['--no-viewer']
            return subprocess.run(command,cwd=ROOT,env=env,timeout=240).returncode
        finally:
            server.terminate()
            try: server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()


if __name__=='__main__':
    raise SystemExit(main())
