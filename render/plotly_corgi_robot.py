#!/usr/bin/env python3
"""CLI wrapper for the interactive Plotly Corgi robot renderer."""

import sys

from legwheel.visualization.plotly_robot import main

if __name__ == "__main__":
    sys.exit(main())
