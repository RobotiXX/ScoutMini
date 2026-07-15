# Copyright 2026 RobotiXX
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import importlib.util
from pathlib import Path

import ament_index_python.packages
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution


def _load_launch_module(monkeypatch):
    monkeypatch.setattr(
        ament_index_python.packages,
        'get_package_share_directory',
        lambda package_name: f'/tmp/{package_name}',
    )
    launch_path = Path(__file__).parents[1] / 'launch' / 'basic.launch.py'
    spec = importlib.util.spec_from_file_location('basic_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(substitutions):
    assert all(isinstance(value, TextSubstitution) for value in substitutions)
    return ''.join(value.text for value in substitutions)


def _forwarded_arguments(include):
    arguments = dict(include.launch_arguments)
    assert all(isinstance(value, LaunchConfiguration)
               for value in arguments.values())
    return {
        name: _text(value.variable_name)
        for name, value in arguments.items()
    }


def test_base_arguments_are_declared_and_forwarded(monkeypatch):
    module = _load_launch_module(monkeypatch)
    description = module.generate_launch_description()
    arguments = {
        action.name: _text(action.default_value)
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert arguments == {
        'use_sim_time': 'false',
        'port_name': 'can2',
        'base_frame': 'base_link',
        'odom_topic_name': 'odom',
        'publish_odom_topic': 'false',
        'publish_odom_tf': 'false',
    }

    includes = [
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert _forwarded_arguments(includes[0]) == {
        name: name for name in arguments
    }
    assert _forwarded_arguments(includes[1]) == {
        'use_sim_time': 'use_sim_time',
    }
