#!/usr/bin/env python3
"""CLI wrapper for the Plotly HTML trajectory viewer."""

import sys

from legwheel.visualization.plotly_csv_viewer import main

if __name__ == "__main__":
    sys.exit(main())
