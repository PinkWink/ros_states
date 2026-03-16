# ROS2 State Observer

A web-based real-time monitoring dashboard for ROS2 (Robot Operating System 2). Observe topics, services, actions, TF trees, and parameters through your browser — without subscribing to or publishing any data.

![Python](https://img.shields.io/badge/Python-3.10+-blue)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green)
![Flask](https://img.shields.io/badge/Flask-3.x-lightgrey)

## Features

- **Topics** — List all active topics with message type, publisher/subscriber count, and Hz measurement
- **Services** — List all services with type and server/client node names
- **Actions** — Detect actions via graph API with type and server/client node names
- **TF Tree** — Subscribe to `/tf` and `/tf_static`, display parent-child frame hierarchy
- **Parameters** — View and modify parameters for any node in real-time
- **Expandable Rows** — Click any row to reveal associated node names (publishers, subscribers, servers, clients)
- **Domain ID Selection** — Set `ROS_DOMAIN_ID` and activate/deactivate monitoring at runtime
- **Auto Refresh** — Topics, Services, Actions, TF update every 5 seconds (configurable) via parallel API calls
- **Manual Refresh** — Click the Refresh button to instantly update data panels; Parameters have a dedicated refresh button
- **ROS2 Package** — Installable via `colcon build` with launch file support

## Architecture

```
Browser (index.html)
    ↕  HTTP JSON (fetch)
Flask Server (app.py)          port 5050 (configurable)
    ↕  Python method call
RosMonitor (ros_monitor.py)    rclpy Graph API + Service Client + TF subscription
    ↕  DDS / rclpy
ROS2 Network                   Topics, Services, Actions, TF, Parameters
```

## Project Structure

```
ros_states/
├── package.xml                 # ROS2 package manifest
├── setup.py                    # ament_python build configuration
├── setup.cfg                   # Script install paths
├── resource/
│   └── ros_states              # ament index marker
├── ros_states/                 # Python module
│   ├── __init__.py
│   ├── app.py                  # Flask web server + REST API endpoints
│   └── ros_monitor.py          # ROS2 monitoring core (rclpy + service client + TF + params)
├── templates/
│   └── index.html              # Dashboard frontend (Flask template)
├── launch/
│   └── ros_states.launch.py    # ROS2 launch file with configurable parameters
├── app.py                      # Legacy entry point (wrapper)
├── ros_monitor.py              # Legacy module (wrapper)
├── run.sh                      # Conda environment activation + launch script
├── README.md                   # This file
├── history.html                # Build history documentation (standalone HTML)
└── architecture.html           # Code architecture diagrams (standalone HTML)
```

## Prerequisites

- **ROS2 Jazzy** installed and sourced
- **Python 3.10+** with `rclpy`, `tf2_ros_py`
- **Conda** (optional, but recommended for environment isolation)

## Installation

### Option 1: As a ROS2 Package (Recommended)

```bash
# Clone into your colcon workspace
cd ~/colcon_ws/src
git clone https://github.com/<your-username>/ros_states.git

# Install Flask
pip install flask

# Build
cd ~/colcon_ws
colcon build --packages-select ros_states
source install/setup.bash
```

### Option 2: Standalone with Conda

```bash
git clone https://github.com/<your-username>/ros_states.git
cd ros_states

# Install Flask in your conda environment
conda run -n ros_jazzy pip install flask
```

## Usage

### Using ROS2 Launch (Recommended)

```bash
# Default: port 5050, 5s interval, opens browser
ros2 launch ros_states ros_states.launch.py

# Custom settings
ros2 launch ros_states ros_states.launch.py port:=8080 update_interval:=3000 open_browser:=false

# All launch arguments
ros2 launch ros_states ros_states.launch.py \
    port:=5050 \
    update_interval:=5000 \
    open_browser:=true \
    domain_id:=0
```

### Using run.sh (Conda)

```bash
chmod +x run.sh

# Default
./run.sh

# With options
./run.sh --port 8080 --update-interval 3000 --open-browser
```

### Manual Start

```bash
# With conda
conda run -n ros_jazzy --no-capture-output python3 app.py --port 5050 --update-interval 5000

# Or with system ROS2
source /opt/ros/jazzy/setup.bash
python3 app.py
```

Then open **http://localhost:5050** in your browser.

### Launch Arguments / CLI Options

| Argument           | Default | Description                           |
|--------------------|---------|---------------------------------------|
| `port`             | 5050    | Web server port number                |
| `update_interval`  | 5000    | Dashboard update interval (ms)        |
| `open_browser`     | true    | Auto-open web browser on launch       |
| `domain_id`        | 0       | Initial ROS_DOMAIN_ID (0-232)         |

### Dashboard Usage

1. Enter a **ROS_DOMAIN_ID** (default: 0)
2. Click **Activate** to start monitoring
3. The dashboard displays five panels:
   - **Topics** — name, message type, Pub/Sub count, Hz
   - **Services** — name, service type
   - **Actions** — name, action type
   - **TF Tree** — parent-child frame hierarchy
   - **Parameters** — select a node, view/edit parameters
4. Click the **arrow (▶)** on any row to expand and see associated node names
5. Click **Refresh (↻)** to instantly update Topics, Services, Actions, and TF panels
6. In the **Parameters** panel:
   - Select a node from the dropdown to view its parameters
   - Click **↻ Nodes** to refresh the node list
   - Click **↻ Params** to refresh parameter values for the selected node
   - Parameters are **not** auto-refreshed — they update only on user action
   - Click **Edit** on any parameter to modify its value
   - Press **Enter** or click **Set** to apply
7. Click **Deactivate** to stop monitoring

### Testing with Demo Nodes

```bash
# Terminal 1: Launch the observer
ros2 launch ros_states ros_states.launch.py

# Terminal 2: Run a talker node
ros2 run demo_nodes_cpp talker

# Terminal 3: Run turtlesim
ros2 run turtlesim turtlesim_node
```

## API Reference

| Method | Endpoint             | Parameters                              | Response                                                    |
|--------|----------------------|-----------------------------------------|-------------------------------------------------------------|
| POST   | `/api/activate`      | `{"domain_id": int}`                    | `{"status":"ok", "domain_id": int}`                         |
| POST   | `/api/deactivate`    | —                                       | `{"status":"ok"}`                                           |
| GET    | `/api/status`        | —                                       | `{"active": bool, "domain_id": int}`                        |
| GET    | `/api/config`        | —                                       | `{"port": int, "update_interval": int}`                     |
| GET    | `/api/topics`        | —                                       | `[{name, type, publishers, subscribers, hz, pub_nodes, sub_nodes}]` |
| GET    | `/api/services`      | —                                       | `[{name, type, server_nodes, client_nodes}]`                |
| GET    | `/api/actions`       | —                                       | `[{name, type, server_nodes, client_nodes}]`                |
| GET    | `/api/tf`            | —                                       | `{"frames":[{child,parent}], "tree":[tree_nodes]}`          |
| GET    | `/api/nodes`         | —                                       | `["node_name", ...]`                                        |
| GET    | `/api/params/list`   | `?node=/node_name`                      | `["param_name", ...]`                                       |
| GET    | `/api/params/get`    | `?node=/node_name&param=param_name`     | `{"type": "String", "value": "hello"}`                      |
| POST   | `/api/params/set`    | `{"node":..., "param":..., "value":...}`| `{"success": bool, "message": "..."}`                       |

## How It Works

- **Graph API** — Uses `rclpy` graph introspection to observe the ROS2 network without subscribing to any topics
- **Hz Measurement** — Spawns `ros2 topic hz` subprocesses sequentially with a 4-second timeout
- **TF Subscription** — Subscribes to `/tf` and `/tf_static` to build a parent-child frame tree
- **Parameter Access** — Uses rclpy service clients (`ListParameters`, `GetParameters`, `SetParameters`) for fast parameter observation and modification without DDS rediscovery
- **Internal Node Filtering** — Excludes `ros_web_monitor` and `_ros2cli_*` nodes from all counts
- **Threading** — 3 threads: Main (Flask), Spin (rclpy callbacks), Hz (subprocess measurement)

## Documentation

Open the standalone HTML files directly in your browser:

- `history.html` — Build history and session logs
- `architecture.html` — System architecture diagrams, data flow, threading model

## License

MIT
