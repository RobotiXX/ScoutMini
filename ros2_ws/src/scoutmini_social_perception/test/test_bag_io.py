"""Tests for corpus metadata and direct rosbag storage helpers."""

from pathlib import Path
import sqlite3

import yaml

from scoutmini_social_perception.bag_io import (
    edge_serialized_messages,
    iter_serialized_messages,
    read_bag_metadata,
    storage_integrity,
    topic_counts,
)


def make_bag(tmp_path: Path) -> Path:
    """Create a minimal sqlite3 rosbag fixture."""
    bag = tmp_path / 'bag'
    bag.mkdir()
    database = bag / 'bag_0.db3'
    with sqlite3.connect(database) as connection:
        connection.execute(
            'CREATE TABLE topics('
            'id INTEGER PRIMARY KEY, name TEXT, type TEXT, '
            'serialization_format TEXT, offered_qos_profiles TEXT)'
        )
        connection.execute(
            'CREATE TABLE messages('
            'id INTEGER PRIMARY KEY, topic_id INTEGER, '
            'timestamp INTEGER, data BLOB)'
        )
        connection.execute(
            "INSERT INTO topics VALUES"
            "(1, '/wanted', 'example/Type', 'cdr', '')"
        )
        connection.execute(
            "INSERT INTO topics VALUES(2, '/other', 'example/Type', 'cdr', '')"
        )
        connection.execute(
            "INSERT INTO messages VALUES(1, 1, 20, X'0102')"
        )
        connection.execute(
            "INSERT INTO messages VALUES(2, 2, 10, X'03')"
        )
    document = {
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3',
            'relative_file_paths': ['bag_0.db3'],
            'topics_with_message_count': [{
                'message_count': 1,
                'topic_metadata': {'name': '/wanted', 'type': 'example/Type'},
            }],
        },
    }
    with (bag / 'metadata.yaml').open('w', encoding='utf-8') as stream:
        yaml.safe_dump(document, stream)
    return bag


def test_metadata_topic_counts_and_filtered_reader(tmp_path):
    bag = make_bag(tmp_path)
    information = read_bag_metadata(bag)
    assert topic_counts(information)['/wanted']['count'] == 1
    records = list(iter_serialized_messages(bag, ['/wanted']))
    assert len(records) == 1
    assert records[0].topic == '/wanted'
    assert records[0].received_ns == 20
    assert records[0].data == b'\x01\x02'
    edges = edge_serialized_messages(bag, ['/wanted'])
    assert [record.received_ns for record in edges['/wanted']] == [20]


def test_storage_integrity_reports_missing_database(tmp_path):
    bag = make_bag(tmp_path)
    ok, problems = storage_integrity(bag)
    assert ok
    assert not problems
    (bag / 'bag_0.db3').unlink()
    ok, problems = storage_integrity(bag)
    assert not ok
    assert problems == ['missing storage file: bag_0.db3']
