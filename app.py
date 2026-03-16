"""Legacy entry point - redirects to ros_states.app.main()."""

import sys
import os

# Add parent directory to path so ros_states package can be found
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_states.app import main

if __name__ == '__main__':
    main()
