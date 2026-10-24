"""Linux foreground session: replace the previous owned service, stop on terminal exit."""
from contextlib import contextmanager
import ctypes
import errno
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.request import ProxyHandler, Request, build_opener

if __package__:
    from .arguments import argument_parser
    from .startup import local_url
else:
    from arguments import argument_parser
    from startup import local_url


def process_identity(pid, app, port):
    """Only accept this user's exact application entry point and requested port."""
    try:
        proc = Path('/proc')/str(pid)
        if pid <= 1 or proc.stat().st_uid != os.getuid():
            return None
        argv = [s.decode() for s in (proc/'cmdline').read_bytes().split(b'\0') if s]
        if not any(Path(s).resolve() == app.resolve() for s in argv[1:] if s.endswith('web_app.py')):
            return None
        actual_port = int(argv[argv.index('--port')+1]) if '--port' in argv else 8765
        if actual_port != port:
            return None
        # /proc/stat's comm field may contain spaces or parentheses.
        fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return fields[19]  # Kernel start ticks; prevents stale PID reuse.
    except (OSError, ValueError, IndexError):
        return None


def owns_listener(pid, port):
    try:
        inodes = set()
        for name in ('tcp', 'tcp6'):
            for row in (Path('/proc/net')/name).read_text().splitlines()[1:]:
                fields = row.split()
                if fields[3] == '0A' and int(fields[1].rsplit(':', 1)[1], 16) == port:
                    inodes.add('socket:['+fields[9]+']')
        for fd in (Path('/proc')/str(pid)/'fd').iterdir():
            try:
                if os.readlink(fd) in inodes:
                    return True
            except OSError:
                pass
    except OSError:
        pass
    return False


def api(url, path, data=None):
    request = Request(url+path, data=json.dumps(data).encode() if data is not None else None,
                      headers={'Content-Type': 'application/json'} if data is not None else {})
    with build_opener(ProxyHandler({})).open(request, timeout=5) as response:
        return json.loads(response.read(65536))


def stop_previous(pid, token, app, port, url):
    if owns_listener(pid, port):
        # Preserve current work and finish diagnostic files before shutdown.
        state = api(url, 'api/v1/action', {'action': 'pause'})
        if state['frames']:
            saved = api(url, 'api/v1/action', {'action': 'save'})
            print('Previous canvas saved: '+saved['url'], flush=True)
        if state['recording']['active']:
            api(url, 'api/v1/action', {'action': 'record_stop'})
    if process_identity(pid, app, port) != token:
        return
    print(f'Stopping previous light painting service (PID {pid})...', flush=True)
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic()+40
    while process_identity(pid, app, port) == token:
        if time.monotonic() >= deadline:
            raise RuntimeError('Previous service has not stopped; a new camera session was not started')
        time.sleep(.1)


@contextmanager
def launch_lock(path):
    import fcntl
    with path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def read_record(path):
    try:
        record = json.loads(path.read_text())
        return record if isinstance(record, dict) else {}
    except (OSError, ValueError):
        return {}


def parent_death_handler(parent_pid):
    def setup():
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGTERM) != 0:  # PR_SET_PDEATHSIG
            raise OSError(ctypes.get_errno(), 'Could not attach service to its launcher')
        if os.getppid() != parent_pid:
            os._exit(1)
    return setup


def main():
    parser = argument_parser(__doc__)
    args = parser.parse_args()
    if not sys.platform.startswith('linux'):
        parser.error('This terminal-session launcher requires Linux')
    if args.listen_fd is not None or not 1 <= args.port <= 65535:
        parser.error('Use a port between 1 and 65535; --listen-fd is reserved for the launcher')
    app = Path(__file__).with_name('web_app.py')
    output = (args.output_dir or app.parent/'output').resolve()
    output.mkdir(parents=True, exist_ok=True)
    record_path, lock_path = output/'.web-instance.json', output/'.web-launch.lock'
    legacy_pid = output.parent/'web.pid'
    url = local_url(args.host, args.port)
    child = listener = None
    terminal_closed = False
    parent_pid = os.getppid()

    def terminate(*_):
        nonlocal terminal_closed
        terminal_closed = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, terminate)
    try:
        with launch_lock(lock_path):
            record = read_record(record_path)
            if (record.get('port') != args.port and type(record.get('pid')) is int
                    and type(record.get('port')) is int
                    and process_identity(record['pid'], app, record['port']) is not None):
                raise RuntimeError('This output directory belongs to another active port; choose a separate --output-dir')
            candidates = []
            if type(record.get('pid')) is int:
                candidates.append((record['pid'], record.get('start_ticks')))
            if legacy_pid.exists():
                try:
                    candidates.append((int(legacy_pid.read_text().strip()), None))
                except ValueError:
                    pass
            seen = set()
            for pid, expected in candidates:
                if pid in seen:
                    continue
                seen.add(pid)
                token = process_identity(pid, app, args.port)
                if token is not None and (expected is None or expected == token):
                    stop_previous(pid, token, app, args.port, url)
            if terminal_closed or os.getppid() != parent_pid:
                return 0
            # Reserve before camera acquisition; pass the bound listener to the child.
            try:
                listener = socket.create_server((args.host, args.port),
                    family=socket.AF_INET6 if ':' in args.host else socket.AF_INET)
            except OSError as exc:
                if exc.errno == errno.EADDRINUSE:
                    raise RuntimeError(f'Port {args.port} is occupied by an untracked service; it was left untouched') from exc
                raise
            command = [sys.executable, '-u', str(app), *sys.argv[1:], '--listen-fd', str(listener.fileno())]
            child = subprocess.Popen(command, pass_fds=(listener.fileno(),),
                                     preexec_fn=parent_death_handler(os.getpid()))
            listener.close()
            listener = None
            token = process_identity(child.pid, app, args.port)
            if token is None:
                raise RuntimeError('New service exited during startup')
            record = dict(pid=child.pid, start_ticks=token, supervisor_pid=os.getpid(), port=args.port)
            temporary = output/f'.web-instance-{os.getpid()}.tmp'
            temporary.write_text(json.dumps(record))
            temporary.replace(record_path)
            if legacy_pid.exists():
                legacy_pid.write_text(str(child.pid))
            print(f'Foreground light painting session. Open: {url}', flush=True)
            print('Keep this terminal open. Ctrl+C or closing the terminal stops the camera service.', flush=True)
        while child.poll() is None:
            if terminal_closed or os.getppid() != parent_pid:
                break
            time.sleep(.1)
        return_code = child.poll()
        return 0 if return_code is None or return_code == 0 or terminal_closed else 1
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f'Light painting launch failed: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if listener:
            listener.close()
        if child is not None:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    print('Camera service has not finished stopping; cleanup is still pending.', file=sys.stderr)
            with launch_lock(lock_path):
                if child.poll() is not None and read_record(record_path).get('pid') == child.pid:
                    record_path.unlink(missing_ok=True)
                    try:
                        if legacy_pid.read_text().strip() == str(child.pid):
                            legacy_pid.unlink()
                    except OSError:
                        pass


if __name__ == '__main__':
    raise SystemExit(main())
