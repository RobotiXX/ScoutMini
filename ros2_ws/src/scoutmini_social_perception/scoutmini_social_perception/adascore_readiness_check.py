"""Report local runtime readiness for AdaSCoRe integration gates."""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
from typing import Any

from ament_index_python.packages import PackageNotFoundError, get_package_prefix


ROS_PACKAGES = [
    'people_msgs',
    'adascore',
    'hunav_msgs',
    'hunav_agent_manager',
    'social_force_window_planner',
    'nav2_controller',
    'tf2_ros',
]

PYTHON_MODULES = [
    'ultralytics',
    'torch',
    'tensorrt',
    'onnx',
    'onnxruntime',
    'sahi',
    'scipy',
]

DEFAULT_MODEL_ARTIFACTS = {
    'yolo_pt': '/home/nvidia/models/yolo/yolo11n.pt',
    'yolo_tensorrt_engine': '/home/nvidia/models/yolo/yolo11n.engine',
}

MODEL_FORMATS = {
    '.pt': 'pytorch',
    '.engine': 'tensorrt',
    '.onnx': 'onnx',
}


def _artifact_status(path: str) -> dict[str, Any]:
    artifact = Path(path).expanduser()
    suffix = artifact.suffix.lower()
    return {
        'path': str(artifact),
        'available': artifact.exists(),
        'format': MODEL_FORMATS.get(suffix, 'unknown'),
        'suffix': suffix,
    }


def _module_status(module_name: str) -> dict[str, Any]:
    try:
        module = importlib.import_module(module_name)
    except ImportError:
        return {'available': False}

    status: dict[str, Any] = {
        'available': True,
        'version': str(getattr(module, '__version__', 'unknown')),
    }
    if module_name == 'torch':
        cuda = getattr(module, 'cuda', None)
        version = getattr(module, 'version', None)
        status['cuda_available'] = bool(cuda.is_available()) if cuda is not None else False
        status['torch_cuda'] = str(getattr(version, 'cuda', None))
    return status


def _ros_package_status(package_name: str) -> dict[str, Any]:
    try:
        prefix = get_package_prefix(package_name)
    except PackageNotFoundError:
        return {'available': False}
    return {'available': True, 'prefix': prefix}


def build_report(
    yolo_pt_path: str = DEFAULT_MODEL_ARTIFACTS['yolo_pt'],
    yolo_engine_path: str = DEFAULT_MODEL_ARTIFACTS['yolo_tensorrt_engine'],
) -> dict[str, Any]:
    python_modules = {name: _module_status(name) for name in PYTHON_MODULES}
    ros_packages = {name: _ros_package_status(name) for name in ROS_PACKAGES}
    model_artifacts = {
        'yolo_pt': _artifact_status(yolo_pt_path),
        'yolo_tensorrt_engine': _artifact_status(yolo_engine_path),
    }
    torch_status = python_modules['torch']
    cuda_pytorch_available = bool(torch_status.get('cuda_available'))
    tensorrt_engine_available = model_artifacts['yolo_tensorrt_engine']['available']
    adascore_deps_ready = all(
        ros_packages[name]['available']
        for name in [
            'people_msgs',
            'adascore',
            'hunav_msgs',
            'hunav_agent_manager',
            'social_force_window_planner',
        ]
    )
    return {
        'ros_packages': ros_packages,
        'python_modules': python_modules,
        'model_artifacts': model_artifacts,
        'summary': {
            'gpu_runtime_detected': cuda_pytorch_available or python_modules['tensorrt']['available'],
            'cuda_pytorch_available': cuda_pytorch_available,
            'tensorrt_available': python_modules['tensorrt']['available'],
            'default_tensorrt_engine_available': tensorrt_engine_available,
            'yolo_gpu_execution_ready': cuda_pytorch_available or tensorrt_engine_available,
            'adascore_dependencies_available': adascore_deps_ready,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description='Report ScoutMini AdaSCoRe readiness gates.')
    parser.add_argument(
        '--yolo-pt-path',
        default=DEFAULT_MODEL_ARTIFACTS['yolo_pt'],
        help='Expected PyTorch YOLO model artifact path.',
    )
    parser.add_argument(
        '--yolo-engine-path',
        default=DEFAULT_MODEL_ARTIFACTS['yolo_tensorrt_engine'],
        help='Expected TensorRT YOLO engine artifact path.',
    )
    args = parser.parse_args()
    report = build_report(args.yolo_pt_path, args.yolo_engine_path)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
