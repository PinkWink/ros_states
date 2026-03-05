"""Flask app for ROS2 State Observer."""

from flask import Flask, jsonify, render_template, request
from ros_monitor import RosMonitor

app = Flask(__name__)
monitor = RosMonitor()


@app.route('/')
def index():
    return render_template('index.html')


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


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5050, debug=False)
