"""Packet-ring slot -> fields of krc_interfaces/ReplyRecord. No ROS imports.

The values come from the ring that the reply process fills after each reply (``server.reply_loop``), so building a
record never touches the reply loop.
"""
from .decider import SRC_OUT
from .shared import F
from .window import STATE_CODE

WIN_NAME = {v: k for k, v in STATE_CODE.items()}


def reply_fields(slot, ov_sent: bool) -> dict:
    """ReplyRecord fields (without the stamp) of one ring slot. ``ov_sent`` is true in closed / step mode."""
    src = int(slot[F["src"]])
    return dict(
        pc_ts_perf=float(slot[F["t_perf"]]),
        ipoc=int(slot[F["ipoc"]]),
        ov_cmd=int(slot[F["ov_sent"]]),
        source=SRC_OUT[src] if 0 <= src < len(SRC_OUT) else "none",
        ov_sent=bool(ov_sent),
        delay=int(slot[F["delay"]]),
        reply_latency_s=float(slot[F["latency_s"]]),
        win=WIN_NAME.get(int(slot[F["win_state"]]), ""),
    )
