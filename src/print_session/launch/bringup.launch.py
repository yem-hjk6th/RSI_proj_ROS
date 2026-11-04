"""Edge stack of one print session: RSI bridge, ELP readout, controller, cameras, recorder and the session node.

    ros2 launch print_session bringup.launch.py mode:=step params_file:=<yaml> readout:=false
    ros2 launch print_session bringup.launch.py mode:=closed params_file:=<yaml> cameras:=<camera yaml>
    ros2 launch print_session bringup.launch.py mode:=closed params_file:=<yaml> image_topic:=/elp/image_raw \\
        start_camera:=true camera_params:=<usb_cam yaml>

cameras:=<file> starts one camera_capture node per section of the file: capture, recording to
<out_dir>/<session>/raw_video/ and the camera's block of session_meta.json (print_session.cameras).
print_session.bore1_readout: true starts width_readout_bore1 (section width_readout_bore1 of params_file; plan A_b1
needs it). If it ends while the session runs and the plan uses bore1, the session ends (legacy run_cl refused A_b1
without the bore1 head); otherwise the session goes on with ELP -> model.
print_session.cage2: true (cage 2, T15) also starts the ELP-B readout (width_readout_elp_b: kind B, section
width_readout_elp_b) and, with cl_controller.bore2 not off, the bore2 readout (width_readout_bore2: camera bore2,
section width_readout_bore2). If bore2 steers (fallback / primary) and its node ends, the session ends.
readout:=false runs without the ELP readout (step mode only). An empty image_topic:= is not accepted by ros2 launch.

mode (shadow | closed | step) and params_file are required and have no default: since 261020 r2, run_cl.py refuses a
missing --mode, because a forgotten mode used to mean 1 % override. Shadow mode is refused unless
print_session.krc_rsi_config is RSI_MIN: the lab's print programs load RSI_OV4, where shadow means 1 % override.
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
from launch.events.process import ProcessExited
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from print_session import topics as T
from print_session.cameras import plan_cameras, switch
from print_session.startup import (BORE1, BORE2, BRIDGE, CONTROLLER, READOUT, READOUT_B, SESSION, Refused,
                                   load_params, plan)


def _setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    mode, params_file = arg("mode"), arg("params_file")
    ses = arg("session") or time.strftime("%Y%m%d_%H%M%S")
    sdir = (Path(arg("out_dir")) / ses).resolve()
    cameras_file = arg("cameras")
    try:
        readout = switch("readout", arg("readout"))
        usb_cam = switch("start_camera", arg("start_camera"))
        preview = switch("preview", arg("preview"))
        image_topic = arg("image_topic") if readout else ""
        params = load_params(params_file)
        p = plan(params, mode, image_topic)
        bore1_topic = params.get(BORE1, {}).get("image_topic", "/bore1/image_raw") if p.bore1 is not None else ""
        extra = {READOUT_B: p.readout_b["image_topic"]} if p.readout_b else {}
        if p.bore2 is not None:
            extra[BORE2] = p.bore2["image_topic"]
        cams = plan_cameras(load_params(cameras_file) if cameras_file else {}, ses, sdir, image_topic, readout,
                            usb_cam, bore1_topic, extra)
    except Refused as e:
        raise RuntimeError(f"print_session refused to start: {e}") from None
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
        if usb_cam:
            camera = "usb_cam"
            cam_params = [arg("camera_params")] if arg("camera_params") else []
            acts.append(Node(package="usb_cam", executable="usb_cam_node_exe", name="elp_camera", output="screen",
                             parameters=cam_params, remappings=[("image_raw", image_topic)]))
    if p.readout_b is not None:
        acts.append(Node(package="width_readout_elp", executable="width_readout_elp_node", name=READOUT_B,
                         output="screen", parameters=[params_file, p.readout_b]))
    for name, over, need in ((BORE1, p.bore1, p.needs_bore1), (BORE2, p.bore2, p.needs_bore2)):
        if over is None:
            continue
        bore = Node(package="width_readout_bore1", executable="width_readout_bore1_node", name=name, output="screen",
                    parameters=[params_file, dict(over, session_meta=str(sdir / "session_meta.json"))])
        acts.append(bore)
        acts.append(RegisterEventHandler(OnProcessExit(
            target_action=bore, on_exit=lambda ev, ctx, need=need, name=name: _bore1_exit(ev, ctx, need, name))))
    if cams:
        camera = "camera_capture"
        cameras_file = str(Path(cameras_file).resolve())
        (sdir / "raw_video").mkdir(exist_ok=True)
        for c in cams:
            acts.append(LogInfo(msg=f"[print_session] camera {c.name}: {c.image_topic}, {c.preview_topic} -> "
                                    f"{c.overrides['video_dir']}"))
            # the recorder drains its queue into the files and closes ffmpeg at the end: give it time
            acts.append(Node(package="camera_capture", executable="camera_node", name=c.name, output="screen",
                             parameters=[cameras_file, c.overrides], sigterm_timeout="30"))
    if preview:
        topics = [c.preview_topic for c in cams][:4] or [f"/{n}/preview" for n in ("elp", "bore1", "elp_b", "bore2")]
        acts.append(Node(package="camera_capture", executable="preview_node", name="camera_preview", output="screen",
                         parameters=[dict(topics=topics + [""] * (4 - len(topics)))]))
    session = Node(package="print_session", executable="session_node", name=SESSION, output="screen",
                   parameters=[params_file, dict(params_file=params_file, mode=mode, session=ses,
                                                 session_dir=str(sdir), image_topic=image_topic, bag=str(bag),
                                                 camera=camera, cameras_file=cameras_file,
                                                 cameras=",".join(c.name for c in cams), readout=readout)])
    acts.append(session)
    acts.append(RegisterEventHandler(OnProcessExit(
        target_action=session, on_exit=[EmitEvent(event=Shutdown(reason="print session ended"))])))
    return acts


def _bore1_exit(ev: ProcessExited, ctx, needed: bool, name: str = BORE1):
    """A bore readout ended. A plan that uses it (bore1 in the order; bore2 fallback / primary) cannot go on without
    it: end the session."""
    cam = "bore2" if name == BORE2 else "bore1"
    msg = f"[print_session] {name} ended (exit code {ev.returncode})"
    if getattr(ctx, "is_shutdown", False):
        return [LogInfo(msg=msg + " at the session end")]
    if not needed:
        return [LogInfo(msg=msg + f"; the session goes on without {cam} (ELP -> model)")]
    return [LogInfo(msg=msg + f"; the plan uses {cam}: ending the session"),
            EmitEvent(event=Shutdown(reason=f"{name} ended"))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", choices=["shadow", "closed", "step"],
                              description="REQUIRED, no default: shadow | closed | step"),
        DeclareLaunchArgument("params_file", description="REQUIRED: parameter yaml (example: config/e1.yaml)"),
        DeclareLaunchArgument("image_topic", default_value="/elp/image_raw",
                              description="ELP image topic for the readout (sensor_msgs/Image, rgb8 / bgr8, "
                                          "stamp = capture time)"),
        DeclareLaunchArgument("readout", default_value="true", choices=["true", "false"],
                              description="false = no ELP readout (step mode only)"),
        DeclareLaunchArgument("cameras", default_value="",
                              description="camera_capture parameter yaml, one section per camera node (example: "
                                          "camera_capture/config/cameras_e1.yaml); records to <session>/raw_video"),
        DeclareLaunchArgument("preview", default_value="false", choices=["true", "false"],
                              description="also open the 2x2 preview window of the cameras' preview topics"),
        DeclareLaunchArgument("start_camera", default_value="false",
                              description="instead of cameras: start usb_cam (ros-jazzy-usb-cam) publishing on "
                                          "image_topic, without recording"),
        DeclareLaunchArgument("camera_params", default_value="", description="usb_cam parameter yaml"),
        DeclareLaunchArgument("out_dir", default_value="sessions", description="sessions are written below this"),
        DeclareLaunchArgument("session", default_value="", description="session name (default: YYYYmmdd_HHMMSS)"),
        OpaqueFunction(function=_setup),
    ])
