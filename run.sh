#!/bin/bash
# ROS2 State Observer launcher
# Usage: ./run.sh [--port PORT] [--update-interval MS] [--open-browser] [--domain-id ID]
#
# Examples:
#   ./run.sh                             # Default: port 5050, 5s interval
#   ./run.sh --port 8080                 # Custom port
#   ./run.sh --open-browser              # Auto-open browser
#   ./run.sh --update-interval 3000      # 3 second refresh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "Starting ROS2 State Observer..."
echo ""

conda run -n ros_jazzy --no-capture-output python3 app.py "$@"
