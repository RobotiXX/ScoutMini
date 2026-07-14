"""Efficient readers for uncompressed sqlite3 rosbag2 recordings."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sqlite3
from typing import Dict, Iterable, Iterator, List, Tuple

import yaml


@dataclass(frozen=True)
class SerializedBagMessage:
    """One serialized message selected directly from rosbag2 storage."""

    topic: str
    type_name: str
    received_ns: int
    data: bytes


def read_bag_metadata(bag: Path) -> Dict[str, object]:
    """Read and validate the common rosbag2 metadata envelope."""
    metadata_path = bag / 'metadata.yaml'
    with metadata_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f'Invalid rosbag metadata: {metadata_path}')
    information = document.get('rosbag2_bagfile_information')
    if not isinstance(information, dict):
        raise ValueError(
            f'Missing rosbag2_bagfile_information: {metadata_path}'
        )
    return information


def topic_counts(
    information: Dict[str, object],
) -> Dict[str, Dict[str, object]]:
    """Return topic metadata indexed by topic name."""
    topics = {}
    for item in information.get('topics_with_message_count', []):
        metadata = item.get('topic_metadata', {})
        name = metadata.get('name')
        if not name:
            continue
        topics[str(name)] = {
            'type': str(metadata.get('type', '')),
            'count': int(item.get('message_count', 0)),
        }
    return topics


def database_paths(bag: Path, information=None) -> List[Path]:
    """Resolve every storage file named by rosbag2 metadata."""
    information = information or read_bag_metadata(bag)
    relative_paths = information.get('relative_file_paths', [])
    return [bag / str(relative_path) for relative_path in relative_paths]


def iter_serialized_messages(
    bag: Path,
    wanted_topics: Iterable[str],
) -> Iterator[SerializedBagMessage]:
    """Yield selected messages without reading unrelated large image topics."""
    information = read_bag_metadata(bag)
    if information.get('storage_identifier') != 'sqlite3':
        raise ValueError(f'Only sqlite3 bags are supported: {bag}')
    wanted = sorted(set(wanted_topics))
    if not wanted:
        return
    placeholders = ','.join('?' for _ in wanted)
    query = (
        'SELECT t.name, t.type, m.timestamp, m.data '
        'FROM messages AS m JOIN topics AS t ON t.id = m.topic_id '
        f'WHERE t.name IN ({placeholders}) ORDER BY m.timestamp'
    )
    for database in database_paths(bag, information):
        uri = f'file:{database}?mode=ro'
        with sqlite3.connect(uri, uri=True) as connection:
            for topic, type_name, timestamp, data in connection.execute(
                query,
                wanted,
            ):
                yield SerializedBagMessage(
                    topic=str(topic),
                    type_name=str(type_name),
                    received_ns=int(timestamp),
                    data=bytes(data),
                )


def edge_serialized_messages(
    bag: Path,
    wanted_topics: Iterable[str],
) -> Dict[str, List[SerializedBagMessage]]:
    """Return the first and last stored message for each requested topic."""
    information = read_bag_metadata(bag)
    if information.get('storage_identifier') != 'sqlite3':
        raise ValueError(f'Only sqlite3 bags are supported: {bag}')
    wanted = sorted(set(wanted_topics))
    candidates = {topic: [] for topic in wanted}
    query = (
        'SELECT t.name, t.type, m.timestamp, m.data '
        'FROM messages AS m JOIN topics AS t ON t.id = m.topic_id '
        'WHERE t.name = ? ORDER BY m.timestamp {} LIMIT 1'
    )
    for database in database_paths(bag, information):
        uri = f'file:{database}?mode=ro'
        with sqlite3.connect(uri, uri=True) as connection:
            for topic in wanted:
                for order in ('ASC', 'DESC'):
                    row = connection.execute(
                        query.format(order),
                        (topic,),
                    ).fetchone()
                    if row is None:
                        continue
                    name, type_name, timestamp, data = row
                    candidates[topic].append(SerializedBagMessage(
                        topic=str(name),
                        type_name=str(type_name),
                        received_ns=int(timestamp),
                        data=bytes(data),
                    ))
    edges = {}
    for topic, records in candidates.items():
        if not records:
            edges[topic] = []
            continue
        ordered = sorted(records, key=lambda record: record.received_ns)
        edges[topic] = [ordered[0]]
        if ordered[-1].received_ns != ordered[0].received_ns:
            edges[topic].append(ordered[-1])
    return edges


def storage_integrity(
    bag: Path,
    information=None,
) -> Tuple[bool, List[str]]:
    """Check storage presence and SQLite integrity without modifying a bag."""
    information = information or read_bag_metadata(bag)
    problems = []
    if information.get('storage_identifier') != 'sqlite3':
        identifier = information.get('storage_identifier')
        problems.append(f'unsupported storage: {identifier}')
        return False, problems
    databases = database_paths(bag, information)
    if not databases:
        problems.append('metadata contains no storage files')
    for database in databases:
        if not database.is_file():
            problems.append(f'missing storage file: {database.name}')
            continue
        try:
            uri = f'file:{database}?mode=ro'
            with sqlite3.connect(uri, uri=True) as connection:
                result = connection.execute('PRAGMA quick_check').fetchone()
            if result != ('ok',):
                problems.append(f'{database.name}: quick_check={result}')
        except sqlite3.Error as exc:
            problems.append(f'{database.name}: {exc}')
    return not problems, problems
