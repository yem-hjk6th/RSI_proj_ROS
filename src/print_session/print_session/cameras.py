"""Cameras of a print session: the bringup's ``cameras`` parameter file and its switches (no ROS imports).

``cameras:=<file>`` is a ROS 2 parameter file with one section per camera node of ``camera_capture`` (examples:
``camera_capture/config/cameras_e1.yaml``, ``cameras_cage.yaml``). The section name is the node name, and so the
topics ``/<name>/image_raw`` and ``/<name>/preview`` unless the section sets ``image_topic`` / ``preview_topic``.
The bringup sets, per camera:
- ``session``: the session name (the file names carry it);
- ``video_dir``: ``<session dir>/raw_video`` (segment files, frame CSVs, recorder sidecars);
- ``session_meta``: ``<session dir>/session_meta.json`` (the camera writes its ``cameras.<name>`` block).

Refused: a camera file and ``start_camera:=true`` (usb_cam) together; a section that sets one of the values above;
two cameras with the same image topic or file names; and, with the readout on, an ``image_topic`` that none of the
cameras publishes (also the bore1 readout's ``image_topic`` when it runs).
"""
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

from cl_controller.controller import Refused

BRINGUP_SET = ("session", "video_dir", "session_meta")
TRUE, FALSE = ("true", "1", "yes", "on"), ("false", "0", "no", "off")


def switch(name: str, value: str) -> bool:
    """A launch argument that must be true or false (``readout``, ``start_camera``, ``preview``)."""
    v = str(value).strip().lower()
    if v in TRUE:
        return True
    if v in FALSE:
        return False
    raise Refused(f"{name}:={value!r}: give true or false")


@dataclass
class CameraNode:
    name: str
    image_topic: str
    preview_topic: str
    overrides: Dict[str, str]           # parameters the bringup passes after the camera file


def plan_cameras(cameras: Dict[str, dict], session: str, session_dir: Path, image_topic: str, readout: bool,
                 usb_cam: bool, bore1_topic: str = "") -> List[CameraNode]:
    """One ``CameraNode`` per section of the camera file (``{node name: ros__parameters}``), or Refused."""
    if not cameras:
        return []
    if usb_cam:
        raise Refused("cameras:= and start_camera:=true both start a camera: use one (start_camera starts usb_cam "
                      "without recording)")
    sdir = Path(session_dir)
    out: List[CameraNode] = []
    seen_topics: Dict[str, str] = {}
    seen_files: Dict[tuple, str] = {}
    for name, p in cameras.items():
        p = p or {}
        bad = [k for k in BRINGUP_SET if k in p]
        if bad:
            raise Refused(f"cameras: {name} sets {', '.join(bad)}; the bringup sets them for the session")
        topic = p.get("image_topic") or f"/{name}/image_raw"
        if topic in seen_topics:
            raise Refused(f"cameras: {name} and {seen_topics[topic]} both publish {topic}")
        seen_topics[topic] = name
        key = (p.get("file_prefix", "elp"), p.get("tag", ""))
        if p.get("record", True) and key in seen_files:
            raise Refused(f"cameras: {name} and {seen_files[key]} would write the same files ({key[0]}_<ses>{key[1]}_"
                          f"...): give one of them a tag")
        seen_files[key] = name
        out.append(CameraNode(name=name, image_topic=topic, preview_topic=p.get("preview_topic") or f"/{name}/preview",
                              overrides=dict(session=session, video_dir=str(sdir / "raw_video"),
                                             session_meta=str(sdir / "session_meta.json"))))
    if readout and image_topic not in seen_topics:
        raise Refused(f"readout on {image_topic}, but no camera in the camera file publishes it (they publish "
                      f"{', '.join(seen_topics)}); set image_topic, or readout:=false (step mode only)")
    if bore1_topic and bore1_topic not in seen_topics:
        raise Refused(f"bore1 readout on {bore1_topic}, but no camera in the camera file publishes it (they publish "
                      f"{', '.join(seen_topics)}); set width_readout_bore1.image_topic")
    return out
