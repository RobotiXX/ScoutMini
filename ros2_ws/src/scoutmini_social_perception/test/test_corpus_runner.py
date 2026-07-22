"""Tests for resumable tracking corpus execution."""

import json

import pytest

from scoutmini_social_perception.corpus_runner import _ensure_configuration


def test_configuration_allows_exact_resume(tmp_path):
    configuration = {'model': '/models/yolo.engine', 'imgsz': 640}

    _ensure_configuration(tmp_path, configuration)
    _ensure_configuration(tmp_path, configuration)

    assert json.loads((tmp_path / 'run_config.json').read_text()) == configuration


def test_configuration_rejects_mixed_results(tmp_path):
    _ensure_configuration(tmp_path, {'imgsz': 640})

    with pytest.raises(RuntimeError, match='Refusing to mix configurations'):
        _ensure_configuration(tmp_path, {'imgsz': 960})
