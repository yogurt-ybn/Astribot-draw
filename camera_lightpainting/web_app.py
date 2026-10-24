"""LAN HTML interface; image processing stays on the camera computer."""
import argparse
import ipaddress
from pathlib import Path
import signal
import socket
import sys
from urllib.parse import urlsplit

from flask import Flask, jsonify, request, send_file

if __package__:
    from .web_controller import ControlError, WebController
    from .startup import PortInUseError, bind_or_reuse
else:
    from web_controller import ControlError, WebController
    from startup import PortInUseError, bind_or_reuse


def create_app(controller):
    app = Flask(__name__, static_folder='web', static_url_path='/assets')
    app.config['MAX_CONTENT_LENGTH'] = 32*1024

    @app.before_request
    def check_request():
        host = (urlsplit(request.host_url).hostname or '').lower()
        try:
            address = ipaddress.ip_address(host)
            allowed = address.is_private or address.is_loopback
        except ValueError:
            allowed = host in {'localhost', socket.gethostname().lower()}
        if not allowed:
            return jsonify(error='请使用这台电脑的局域网 IP 地址访问'), 403
        if request.method == 'POST':
            origin = request.headers.get('Origin')
            if origin and urlsplit(origin).netloc != request.host:
                return jsonify(error='不允许跨站控制请求'), 403
            if not request.is_json or not isinstance(request.get_json(silent=True), dict):
                return jsonify(error='请求必须为 JSON 对象'), 400

    @app.after_request
    def response_headers(response):
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['X-Frame-Options'] = 'DENY'
        return response

    @app.errorhandler(ControlError)
    def control_error(error):
        return jsonify(error=str(error)), 409

    @app.get('/')
    def index():
        return send_file(Path(__file__).parent/'web/index.html')

    @app.get('/api/v1/state')
    @app.get('/api/state')
    def state():
        return jsonify(controller.state())

    @app.get('/api/v1/devices')
    @app.get('/api/devices')
    def devices():
        choices = [dict(value=str(i), name='相机 '+str(i)) for i in range(4)]
        linux = Path('/sys/class/video4linux')
        if linux.exists():
            choices = [dict(value='/dev/'+p.name, name=(p/'name').read_text().strip()+' · '+p.name)
                       for p in sorted(linux.glob('video*'))]
        current = str(controller.source)
        if current not in [item['value'] for item in choices]:
            choices.insert(0, dict(value=current, name='当前相机 · '+current))
        return jsonify(devices=choices)

    @app.get('/api/v1/preview/<view>.jpg')
    @app.get('/preview/<view>.jpg')
    def preview(view):
        if view not in {'camera', 'result', 'mask', 'selected'}:
            return jsonify(error='未知预览'), 404
        return app.response_class(controller.preview(view), mimetype='image/jpeg')

    @app.post('/api/v1/settings')
    @app.post('/api/settings')
    def settings():
        controller.update_settings(request.get_json())
        return jsonify(controller.state())

    @app.post('/api/v1/action')
    @app.post('/api/action')
    def action():
        data = request.get_json()
        name = data.get('action')
        if name == 'start':
            controller.set_drawing(True)
        elif name == 'pause':
            controller.set_drawing(False)
        elif name == 'clear':
            controller.clear()
        elif name == 'background':
            controller.capture_background()
        elif name == 'roi':
            controller.set_roi(data.get('coordinates'))
        elif name == 'connect':
            source = data.get('source')
            if not isinstance(source, str) or not (source.isdecimal() and int(source) < 16 or source.startswith('/dev/') and Path(source).exists()):
                raise ControlError('请选择有效的相机编号或设备')
            controller.connect(int(source) if source.isdecimal() else source)
        elif name == 'save':
            return jsonify(controller.save())
        elif name == 'record_start':
            controller.start_recording()
        elif name == 'record_stop':
            controller.stop_recording()
        else:
            return jsonify(error='未知操作'), 400
        return jsonify(controller.state())

    @app.get('/api/v1/frame')
    def frame():
        return jsonify(controller.frame_metadata())

    @app.get('/api/v1/result.png')
    def result_png():
        result, metadata = controller.result_snapshot()
        response = app.response_class(controller.encode_png(result), mimetype='image/png')
        response.headers['X-Frame-Sequence'] = str(metadata.get('sequence', ''))
        response.headers['X-Captured-Ns'] = str(metadata.get('captured_ns', ''))
        return response

    @app.get('/api/v1/capabilities')
    def capabilities():
        return jsonify(application='astribot-lightpainting', api_version='1', modes=['neon', 'exposure'], light_colors=['cyan', 'white'],
                       previews=['camera', 'selected', 'mask', 'result'],
                       overlay_mode='last_drawing_frame', overlay_brightness_range=[0, 100],
                       frame_format='RGB uint8 HxWx3', mask_values=[0, 1],
                       coordinate_space='processed_frame', timestamp_clock='monotonic_ns',
                       endpoints=dict(state='/api/v1/state', settings='/api/v1/settings',
                                      action='/api/v1/action', frame='/api/v1/frame',
                                      result='/api/v1/result.png', devices='/api/v1/devices'))

    @app.get('/files/<path:name>')
    def download(name):
        path = (controller.output_dir/name).resolve()
        if (not path.is_relative_to(controller.output_dir) or not path.is_file()
                or path.suffix not in {'.png', '.zip'} or name.split('/')[0] not in {'captures', 'diagnostics'}):
            return jsonify(error='文件不存在'), 404
        return send_file(path, as_attachment=True, download_name=path.name)

    return app


def main():
    from waitress import create_server
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', default=8765, type=int)
    parser.add_argument('--camera', default='0')
    parser.add_argument('--video')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--diagnostic', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    try:
        listener, url = bind_or_reuse(args.host, args.port)
    except (PortInUseError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    if listener is None:
        print(f'Light painting is already running. Open: {url}', flush=True)
        print('Camera and settings can be managed in the web page.', flush=True)
        return 0
    source = args.video if args.video else int(args.camera) if args.camera.isdecimal() else args.camera
    controller = server = None
    def shutdown(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        controller = WebController(source, args.output_dir)
        server = create_server(create_app(controller), sockets=[listener], threads=8,
                               max_request_body_size=32*1024, channel_timeout=20)
        print(f'Light painting web: {url}', flush=True)
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.close()
            server.task_dispatcher.shutdown()
        listener.close()
        if controller:
            controller.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
