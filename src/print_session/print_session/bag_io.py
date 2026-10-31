"""rosbag2 read / write helpers (rosbag2_py)."""
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, Optional, Sequence, Tuple

from . import topics as T


def _msg_class(type_name: str):
    from rosidl_runtime_py.utilities import get_message
    return get_message(type_name)


def read_bag(uri, topics: Optional[Sequence[str]] = None) -> Dict[str, list]:
    """{topic: [message, ...]} in bag (receive-time) order; only `topics` when given."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    cls = {name: _msg_class(tp) for name, tp in types.items() if topics is None or name in topics}
    out: Dict[str, list] = defaultdict(list)
    while reader.has_next():
        topic, data, _t = reader.read_next()
        out[topic].append(deserialize_message(data, cls[topic]))
    return dict(out)


def write_bag(uri, items: Iterable[Tuple[str, object, int]], storage_id: str = T.STORAGE_ID) -> int:
    """Write (topic, message, receive time ns) items, in the given order, into a new bag. Returns the count."""
    import rosbag2_py
    from rclpy.serialization import serialize_message

    if Path(uri).exists():
        raise FileExistsError(uri)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    made = set()
    n = 0
    for topic, msg, t_ns in items:
        if topic not in made:
            writer.create_topic(rosbag2_py.TopicMetadata(id=len(made), name=topic, type=T.TYPES[topic],
                                                         serialization_format="cdr"))
            made.add(topic)
        writer.write(topic, serialize_message(msg), int(t_ns))
        n += 1
    del writer
    return n
