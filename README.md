# ROS2 State Observer

A web-based real-time monitoring dashboard for ROS2 (Robot Operating System 2). Observe topics, services, actions, and TF trees through your browser — without subscribing to or publishing any data.

![Python](https://img.shields.io/badge/Python-3.10+-blue)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green)
![Flask](https://img.shields.io/badge/Flask-3.x-lightgrey)

## Features

- **Topics** — List all active topics with message type, publisher/subscriber count, and Hz measurement
- **Services** — List all services with type and server/client node names
- **Actions** — Detect actions via graph API with type and server/client node names
- **TF Tree** — Subscribe to `/tf` and `/tf_static`, display parent-child frame hierarchy
- **Expandable Rows** — Click any row to reveal associated node names (publishers, subscribers, servers, clients)
- **Domain ID Selection** — Set `ROS_DOMAIN_ID` and activate/deactivate monitoring at runtime
- **Auto Refresh** — Dashboard updates every 5 seconds via parallel API calls
- **Manual Refresh** — Click the Refresh button to instantly update all panels without waiting for the next auto-refresh cycle

## Architecture

```
Browser (index.html)
    ↕  HTTP JSON (fetch)
Flask Server (app.py)          port 5050
    ↕  Python method call
RosMonitor (ros_monitor.py)    rclpy Graph API + subprocess + TF subscription
    ↕  DDS / rclpy
ROS2 Network                   Topics, Services, Actions, TF
```

## Project Structure

```
ros_states/
├── app.py                  # Flask web server + REST API endpoints (port 5050)
├── ros_monitor.py          # ROS2 monitoring core (rclpy + subprocess + TF)
├── run.sh                  # Conda environment activation + launch script
├── README.md               # This file
├── history.html            # Build history documentation (standalone HTML)
├── architecture.html       # Code architecture diagrams (standalone HTML)
└── templates/
    └── index.html          # Dashboard frontend (Flask template)
```

## Prerequisites

- **ROS2 Jazzy** installed and sourced
- **Python 3.10+** with `rclpy`, `tf2_ros_py`
- **Conda** (optional, but recommended for environment isolation)

## Installation

### Option 1: Using Conda (Recommended)

```bash
# Clone the repository
git clone https://github.com/<your-username>/ros_states.git
cd ros_states

# Create conda environment with ROS2 Jazzy (if not already set up)
# Ensure rclpy and tf2_ros_py are available in your conda environment

# Install Flask
conda run -n ros_jazzy pip install flask
```

### Option 2: Using System ROS2

```bash
# Clone the repository
git clone https://github.com/<your-username>/ros_states.git
cd ros_states

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install Flask
pip install flask
```

## Usage

### Quick Start

```bash
# Using the launch script (conda)
chmod +x run.sh
./run.sh
```

### Manual Start

```bash
# With conda
conda run -n ros_jazzy --no-capture-output python3 app.py

# Or with system ROS2
source /opt/ros/jazzy/setup.bash
python3 app.py
```

Then open **http://localhost:5050** in your browser.

### Dashboard Usage

1. Enter a **ROS_DOMAIN_ID** (default: 0)
2. Click **Activate** to start monitoring
3. The dashboard displays four panels:
   - **Topics** — name, message type, Pub/Sub count, Hz
   - **Services** — name, service type
   - **Actions** — name, action type
   - **TF Tree** — parent-child frame hierarchy
4. Click the **arrow (▶)** on any row to expand and see associated node names
5. Click **Refresh (↻)** to instantly update all panels (resets the 5-second auto-refresh timer)
6. Click **Deactivate** to stop monitoring

### Testing with Demo Nodes

```bash
# Terminal 1: Start the observer
./run.sh

# Terminal 2: Run a talker node
ros2 run demo_nodes_cpp talker

# Terminal 3: Run turtlesim
ros2 run turtlesim turtlesim_node
```

## API Reference

| Method | Endpoint           | Parameters              | Response                                                    |
|--------|--------------------|-------------------------|-------------------------------------------------------------|
| POST   | `/api/activate`    | `{"domain_id": int}`    | `{"status":"ok", "domain_id": int}`                         |
| POST   | `/api/deactivate`  | —                       | `{"status":"ok"}`                                           |
| GET    | `/api/status`      | —                       | `{"active": bool, "domain_id": int}`                        |
| GET    | `/api/topics`      | —                       | `[{name, type, publishers, subscribers, hz, pub_nodes, sub_nodes}]` |
| GET    | `/api/services`    | —                       | `[{name, type, server_nodes, client_nodes}]`                |
| GET    | `/api/actions`     | —                       | `[{name, type, server_nodes, client_nodes}]`                |
| GET    | `/api/tf`          | —                       | `{"frames":[{child,parent}], "tree":[tree_nodes]}`          |

## How It Works

- **Graph API** — Uses `rclpy` graph introspection (e.g., `get_topic_names_and_types()`, `get_publishers_info_by_topic()`) to observe the ROS2 network without subscribing to any topics
- **Hz Measurement** — Spawns `ros2 topic hz` subprocesses sequentially with a 4-second timeout, then parses the output
- **TF Subscription** — Subscribes to `/tf` and `/tf_static` to build a parent-child frame tree
- **Internal Node Filtering** — Excludes `ros_web_monitor` and `_ros2cli_*` nodes from all counts and listings
- **Threading** — 3 threads: Main (Flask), Spin (rclpy callbacks), Hz (subprocess measurement)

## Documentation

Open the standalone HTML files directly in your browser:

- `history.html` — Build history and session logs
- `architecture.html` — System architecture diagrams, data flow, threading model

## License

MIT
