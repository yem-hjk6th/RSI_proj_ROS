"""Edge stack of one print session: RSI bridge, ELP readout, controller, recorder and the session node.

    ros2 launch print_session bringup.launch.py mode:=shadow params_file:=config/e1.yaml
    ros2 launch print_session bringup.launch.py mode:=closed params_file:=<yaml> image_topic:=/elp/image_raw \\
        start_camera:=true camera_params:=<usb_cam yaml>

mode (shadow | closed | step) and params_file are required and have no default: since 261020 r2, run_cl.py refuses a
missing --mode, because a forgotten mode used to mean 1 % override.
The start-up checks run before any node starts (print_session.startup). A refusal stops the launch with its reason.
The session ends when RSI disconnects (or on Ctrl+C); the recorder is then stopped and the bag closed.
Output: <out_dir>/<session>/session_meta.json and <out_dir>/<session>/bag (mcap).
"""
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from print_session import topics as T
from print_session.startup import BRIDGE, CONTROLLER, READOUT, SESSION, Refused, load_params, plan


def _setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    mode, params_file = arg("mode"), arg("params_file")
    image_topic = arg("image_topic")
    try:
        p = plan(load_params(params_file), mode, image_topic)
    except Refused as e:
        raise RuntimeError(f"print_session refused to start: {e}") from None
    ses = arg("session") or time.strftime("%Y%m%d_%H%M%S")
    sdir = (Path(arg("out_dir")) / ses).resolve()
    sdir.mkdir(parents=True, exist_ok=True)
    bag = sdir / "bag"
    params_file = str(Path(params_file).resolve())
    acts = [LogInfo(msg=f"[print_session] {line}") for line in p.log]
    acts.append(LogInfo(msg=f"[print_session] session {ses} mode={mode} -> {sdir}"))
    acts.append(ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", T.STORAGE_ID, "-o", str(bag), "--regex", T.RECORD_REGEX,
             "--include-unpublished-topics", "--disable-keyboard-controls"],
        name="recorder", output="screen", sigterm_timeout="10"))
    acts.append(Node(package="krc_rsi_bridge", executable="bridge_node", name=BRIDGE, output="screen",
                     parameters=[params_file, p.bridge]))
    acts.append(Node(package="cl_controller", executable="controller_node", name=CONTROLLER, output="screen",
                     parameters=[params_file, p.controller]))
    camera = ""
    if p.readout is not None:
        acts.append(Node(package="width_readout_elp", executable="width_readout_elp_node", name=READOUT,
                         output="screen", parameters=[params_file, p.readout]))
        if arg("start_camera").lower() in ("1", "true", "yes"):
            camera = "usb_cam"
            cam_params = [arg("camera_params")] if arg("camera_params") else []
            acts.append(Node(package="usb_cam", executable="usb_cam_node_exe", name="elp_camera", output="screen",
                             parameters=cam_params, remappings=[("image_raw", image_topic)]))
    session = Node(package="print_session", executable="session_node", name=SESSION, output="screen",
                   parameters=[params_file, dict(params_file=params_file, mode=mode, session=ses,
                                                 session_dir=str(sdir), image_topic=image_topic, bag=str(bag),
                                                 camera=camera)])
    acts.append(session)
    acts.append(RegisterEventHandler(OnProcessExit(
        target_action=session, on_exit=[EmitEvent(event=Shutdown(reason="print session ended"))])))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", choices=["shadow", "closed", "step"],
                              description="REQUIRED, no default: shadow | closed | step"),
        DeclareLaunchArgument("params_file", description="REQUIRED: parameter yaml (example: config/e1.yaml)"),
        DeclareLaunchArgument("image_topic", default_value="/elp/image_raw",
                              description="ELP image topic for the readout (sensor_msgs/Image, rgb8 / bgr8, "
                                          "stamp = capture time); empty = no readout (step mode only)"),
        DeclareLaunchArgument("start_camera", default_value="false",
                              description="also start usb_cam (ros-jazzy-usb-cam) publishing on image_topic"),
        DeclareLaunchArgument("camera_params", default_value="", description="usb_cam parameter yaml"),
        DeclareLaunchArgument("out_dir", default_value="sessions", description="sessions are written below this"),
        DeclareLaunchArgument("session", default_value="", description="session name (default: YYYYmmdd_HHMMSS)"),
        OpaqueFunction(function=_setup),
    ])
