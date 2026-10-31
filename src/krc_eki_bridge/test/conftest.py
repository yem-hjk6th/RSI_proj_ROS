import os
import sys

# make the package importable when pytest runs from the source tree (no colcon install)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
