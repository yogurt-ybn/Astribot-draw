from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import socket
import threading
import unittest
from unittest.mock import patch

from startup import PortInUseError, bind_or_reuse
from web_app import main


class StartupTests(unittest.TestCase):
    def test_port_is_reserved_before_camera_creation(self):
        listener, _ = bind_or_reuse('127.0.0.1', 0)
        self.addCleanup(listener.close)
        with socket.socket() as probe:
            with self.assertRaises(OSError):
                probe.bind(listener.getsockname())

    def server(self, payload):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps(payload).encode())

            def log_message(self, *_):
                pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server.server_port

    def test_existing_instance_is_reused_without_opening_camera(self):
        port = self.server(dict(api_version='1', frame_format='RGB uint8 HxWx3',
                                endpoints=dict(state='/api/v1/state', result='/api/v1/result.png')))
        with patch('sys.argv', ['web_app.py', '--host', '127.0.0.1', '--port', str(port)]), \
                patch('web_app.WebController') as controller:
            self.assertEqual(main(), 0)
            controller.assert_not_called()

    def test_unrelated_service_is_not_reused_or_stopped(self):
        port = self.server(dict(application='other'))
        with self.assertRaises(PortInUseError):
            bind_or_reuse('127.0.0.1', port)
        with patch('sys.argv', ['web_app.py', '--host', '127.0.0.1', '--port', str(port)]), \
                patch('web_app.WebController') as controller:
            self.assertEqual(main(), 1)
            controller.assert_not_called()
