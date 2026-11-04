"""The package is plain Python: no ROS import anywhere in path_geometry/."""
import ast
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "path_geometry"
ROS = ("rclpy", "ament_index_python", "rosidl", "krc_interfaces", "builtin_interfaces", "std_msgs", "sensor_msgs")


def test_no_ros_imports():
    for f in sorted(PKG.glob("*.py")):
        for node in ast.walk(ast.parse(f.read_text(encoding="utf-8"))):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                names = [node.module or ""]
            for n in names:
                assert n.split(".")[0] not in ROS, f"{f.name}: imports {n}"


def test_geometry_numpy_only():
    tree = ast.parse((PKG / "geometry.py").read_text(encoding="utf-8"))
    mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    mods |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 0}
    assert mods <= {"__future__", "csv", "json", "math", "dataclasses", "pathlib", "typing", "numpy"}, mods
