#!/bin/bash
# ROS2 State Observer launcher
# Usage: ./run.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "Starting ROS2 State Observer..."
echo "Open http://localhost:5050 in your browser"
echo ""

conda run -n ros_jazzy --no-capture-output python3 app.py
