"""Flask app for ROS2 State Observer."""

import argparse
import os
import webbrowser
import threading

from flask import Flask, jsonify, render_template, request
from ros_states.ros_monitor import RosMonitor

monitor = RosMonitor()

# Default config
_config = {
    'port': 5050,
    'update_interval': 5000,
}


def create_app():
    # Resolve template directory: check installed share path, fallback to local
    template_dir = None
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('ros_states')
        share_templates = os.path.join(pkg_share, 'templates')
        if os.path.isdir(share_templates):
            template_dir = share_templates
    except Exception:
        pass

    if template_dir is None:
        # Fallback: look relative to this file's parent (project root)
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        local_templates = os.path.join(project_root, 'templates')
        if os.path.isdir(local_templates):
            template_dir = local_templates

    app = Flask(__name__, template_folder=template_dir)

    @app.route('/')
    def index():
        return render_template('index.html', update_interval=_config['update_interval'])

    @app.route('/api/activate', methods=['POST'])
    def activate():
        data = request.get_json()
        domain_id = data.get('domain_id', 0)
        try:
            monitor.activate(domain_id)
            return jsonify({'status': 'ok', 'domain_id': monitor.domain_id})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    @app.route('/api/deactivate', methods=['POST'])
    def deactivate():
        try:
            monitor.deactivate()
            return jsonify({'status': 'ok'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    @app.route('/api/status')
    def status():
        return jsonify({
            'active': monitor.active,
            'domain_id': monitor.domain_id,
        })

    @app.route('/api/config')
    def config():
        return jsonify(_config)

    @app.route('/api/topics')
    def topics():
        return jsonify(monitor.get_topics())

    @app.route('/api/services')
    def services():
        return jsonify(monitor.get_services())

    @app.route('/api/actions')
    def actions():
        return jsonify(monitor.get_actions())

    @app.route('/api/tf')
    def tf():
        return jsonify(monitor.get_tf_tree())

    # --- Parameter API ---

    @app.route('/api/nodes')
    def nodes():
        return jsonify(monitor.get_node_list())

    @app.route('/api/params/list')
    def param_list():
        node_name = request.args.get('node', '')
        if not node_name:
            return jsonify([])
        return jsonify(monitor.get_node_parameters(node_name))

    @app.route('/api/params/get')
    def param_get():
        node_name = request.args.get('node', '')
        param_name = request.args.get('param', '')
        if not node_name or not param_name:
            return jsonify({'error': 'Missing node or param'}), 400
        result = monitor.get_parameter_value(node_name, param_name)
        if result is None:
            return jsonify({'error': 'Could not get parameter'}), 404
        return jsonify(result)

    @app.route('/api/params/set', methods=['POST'])
    def param_set():
        data = request.get_json()
        node_name = data.get('node', '')
        param_name = data.get('param', '')
        value = data.get('value', '')
        if not node_name or not param_name:
            return jsonify({'success': False, 'message': 'Missing node or param'}), 400
        result = monitor.set_parameter_value(node_name, param_name, value)
        return jsonify(result)

    return app


def main():
    parser = argparse.ArgumentParser(description='ROS2 State Observer Web Server')
    parser.add_argument('--port', type=int, default=5050, help='Web server port (default: 5050)')
    parser.add_argument('--update-interval', type=int, default=5000,
                        help='Dashboard update interval in ms (default: 5000)')
    parser.add_argument('--open-browser', action='store_true', default=False,
                        help='Open web browser on startup')
    parser.add_argument('--domain-id', type=int, default=0,
                        help='Initial ROS_DOMAIN_ID (default: 0)')
    args = parser.parse_args()

    _config['port'] = args.port
    _config['update_interval'] = args.update_interval

    app = create_app()

    if args.open_browser:
        def _delayed_open():
            import time
            time.sleep(1.5)
            webbrowser.open(f'http://localhost:{args.port}')
        threading.Thread(target=_delayed_open, daemon=True).start()

    print(f'Starting ROS2 State Observer on port {args.port}')
    print(f'Update interval: {args.update_interval}ms')
    print(f'Open http://localhost:{args.port} in your browser')
    app.run(host='0.0.0.0', port=args.port, debug=False)


if __name__ == '__main__':
    main()
