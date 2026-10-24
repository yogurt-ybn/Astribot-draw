"""Reserve the HTTP port before acquiring a camera; reuse a healthy instance."""
import errno
import json
import socket
from urllib.request import ProxyHandler, build_opener


class PortInUseError(RuntimeError):
    pass


def local_url(host, port):
    host = {'0.0.0.0': '127.0.0.1', '::': '::1'}.get(host, host)
    return f'http://{"["+host+"]" if ":" in host else host}:{port}/'


def is_lightpainting_service(url):
    try:
        with build_opener(ProxyHandler({})).open(url+'api/v1/capabilities', timeout=2) as response:
            payload = json.loads(response.read(65536))
        return (isinstance(payload, dict) and payload.get('api_version') == '1'
                and payload.get('frame_format') == 'RGB uint8 HxWx3'
                and payload.get('endpoints', {}).get('state') == '/api/v1/state'
                and payload.get('endpoints', {}).get('result') == '/api/v1/result.png')
    except (OSError, ValueError, AttributeError):
        return False


def bind_or_reuse(host, port):
    """Return (reserved socket, URL), or (None, URL) for the existing service.

    A different process is never stopped. Holding the socket closes the usual
    port-check/start race, including simultaneous launches of this application.
    """
    url = local_url(host, port)
    try:
        listener = socket.create_server((host, port), family=socket.AF_INET6 if ':' in host else socket.AF_INET)
    except OSError as exc:
        if exc.errno not in (errno.EADDRINUSE, 10048):
            raise
        if is_lightpainting_service(url):
            return None, url
        raise PortInUseError(f'Port {port} is occupied by another service. Choose a free --port.') from exc
    return listener, url
