"""Linux lifecycle checks use a generated video, never a physical camera."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from urllib.request import urlopen

import cv2
import numpy as np


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux foreground/PTY lifecycle')
class SessionLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.video = self.folder/'input.avi'
        writer = cv2.VideoWriter(str(self.video), cv2.VideoWriter_fourcc(*'MJPG'), 30, (80, 60))
        self.assertTrue(writer.isOpened())
        writer.write(np.zeros((60, 80, 3), np.uint8))
        writer.release()
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            self.port = probe.getsockname()[1]
        self.url = f'http://127.0.0.1:{self.port}/api/v1/capabilities'
        self.script = Path(__file__).resolve().parents[1]/'session_launcher.py'
        self.output = self.folder/'output'

    def command(self):
        return [sys.executable, '-u', str(self.script), '--host', '127.0.0.1',
                '--port', str(self.port), '--video', str(self.video), '--output-dir', str(self.output)]

    @staticmethod
    def stop(process):
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=15)

    def launch(self):
        log = (self.folder/f'launcher-{time.time_ns()}.log').open('wb')
        self.addCleanup(log.close)
        process = subprocess.Popen(self.command(), stdin=subprocess.DEVNULL, stdout=log, stderr=log)
        self.addCleanup(self.stop, process)
        return process

    def wait_ready(self, previous=None):
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            try:
                with urlopen(self.url, timeout=.3) as response:
                    state = json.load(response)
                if state['process_id'] != previous:
                    return state['process_id']
            except (OSError, ValueError):
                pass
            time.sleep(.05)
        self.fail('service did not become ready')

    def wait_closed(self):
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=.2):
                    pass
            except OSError:
                return
            time.sleep(.05)
        self.fail('service still listens after terminal/launcher exit')

    def test_second_launch_stops_old_service_and_keeps_new_pid_record(self):
        first = self.launch()
        old_pid = self.wait_ready()
        second = self.launch()
        new_pid = self.wait_ready(previous=old_pid)
        first.wait(timeout=10)
        self.assertIsNone(second.poll())
        record = json.loads((self.output/'.web-instance.json').read_text())
        self.assertEqual(record['pid'], new_pid)
        self.stop(second)
        self.wait_closed()
        self.assertFalse((self.output/'.web-instance.json').exists())

    def test_closing_controlling_terminal_stops_child(self):
        import fcntl
        import pty
        import termios
        master, slave = pty.openpty()

        def attach_terminal():
            os.setsid()
            fcntl.ioctl(slave, termios.TIOCSCTTY, 0)

        process = subprocess.Popen(self.command(), stdin=slave, stdout=slave, stderr=slave,
                                   preexec_fn=attach_terminal)
        os.close(slave)
        self.addCleanup(self.stop, process)
        try:
            self.wait_ready()
        finally:
            os.close(master)  # Actual PTY hangup, not a synthetic SIGTERM.
        process.wait(timeout=12)
        self.wait_closed()

    def test_launcher_death_does_not_orphan_service(self):
        process = self.launch()
        self.wait_ready()
        process.kill()  # Kernel parent-death signal must stop the child as well.
        process.wait(timeout=5)
        self.wait_closed()

    def test_unrelated_http_service_is_left_running(self):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'other application')
            def log_message(self, *_):
                pass
        server = ThreadingHTTPServer(('127.0.0.1', self.port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        result = subprocess.run(self.command(), capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('untracked service', result.stderr)
        with urlopen(self.url, timeout=2) as response:
            self.assertEqual(response.read(), b'other application')
