"""Run resumable, motor-isolated tracking validation over a corpus manifest."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory


def _write_summary(output: Path, runs, failures, configuration) -> None:
    summary = {
        'completed_runs': len(runs),
        'failed_runs': len(failures),
        'runs': runs,
        'failures': failures,
        'configuration': configuration,
        'aggregate': {
            'frames': sum(run.get('frames', 0) for run in runs),
            'detections': sum(run.get('detections', 0) for run in runs),
            'candidate_events': sum(
                run.get('candidate_events', 0) for run in runs
            ),
            'baseline_association_changes': sum(
                run.get('baseline_association_changes', 0) for run in runs
            ),
        },
    }
    with (output / 'corpus_results.json').open(
        'w', encoding='utf-8'
    ) as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write('\n')
    fields = [
        'corpus_id', 'segment_id', 'scene', 'split', 'frames', 'detections',
        'unique_public_ids', 'singleton_public_ids',
        'suspected_fragmentations', 'baseline_association_changes',
        'mean_matched_iou', 'review_reel',
    ]
    with (output / 'corpus_results.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=fields,
            extrasaction='ignore',
        )
        writer.writeheader()
        writer.writerows(runs)


def _ensure_configuration(output: Path, configuration) -> None:
    """Create an immutable run configuration or verify an exact resume."""
    path = output / 'run_config.json'
    if path.is_file():
        with path.open(encoding='utf-8') as stream:
            existing = json.load(stream)
        if existing != configuration:
            raise RuntimeError(
                f'Refusing to mix configurations in result directory: {path}'
            )
        return
    with path.open('w', encoding='utf-8') as stream:
        json.dump(configuration, stream, indent=2, sort_keys=True)
        stream.write('\n')


def run(args) -> int:
    """Execute or resume selected manifest segments."""
    manifest_path = Path(args.manifest).resolve()
    with manifest_path.open(encoding='utf-8') as stream:
        manifest = json.load(stream)
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    configuration = {
        'confidence_threshold': args.confidence_threshold,
        'imgsz': args.imgsz,
        'iou_threshold': args.iou_threshold,
        'model': str(Path(args.model).expanduser().resolve()),
        'rate': args.rate,
        'reid_model': str(Path(args.reid_model).expanduser().resolve()),
        'target_fps': args.target_fps,
        'tracker_config': (
            str(Path(args.tracker_config).expanduser().resolve())
            if args.tracker_config else ''
        ),
    }
    _ensure_configuration(output, configuration)
    script = Path(args.script).resolve() if args.script else Path(
        get_package_share_directory('scoutmini_social_perception')
    ) / 'scripts' / 'run_tracking_bag.sh'
    environment = os.environ.copy()
    environment.update({
        'SCOUTMINI_YOLO_MODEL': args.model,
        'SCOUTMINI_REID_MODEL': args.reid_model,
        'SCOUTMINI_PLAYBACK_RATE': str(args.rate),
        'SCOUTMINI_TARGET_FPS': str(args.target_fps),
        'SCOUTMINI_IMGSZ': str(args.imgsz),
        'SCOUTMINI_CONFIDENCE_THRESHOLD': str(args.confidence_threshold),
        'SCOUTMINI_IOU_THRESHOLD': str(args.iou_threshold),
    })
    if args.tracker_config:
        environment['SCOUTMINI_TRACKER_CONFIG'] = args.tracker_config

    runs = []
    failures = []
    attempted = 0
    selected_names = set(args.bag_name)
    for bag in manifest['bags']:
        if args.split != 'all' and bag['split'] != args.split:
            continue
        if selected_names and bag['name'] not in selected_names:
            continue
        for segment in bag.get('segments', []):
            if args.max_runs and attempted >= args.max_runs:
                _write_summary(output, runs, failures, configuration)
                return int(bool(failures))
            attempted += 1
            run_dir = output / bag['corpus_id'] / segment['id']
            metrics_path = (
                run_dir / f"{bag['name']}_tracking_metrics.json"
            )
            metadata = {
                'corpus_id': bag['corpus_id'],
                'segment_id': segment['id'],
                'split': bag['split'],
            }
            if metrics_path.is_file():
                with metrics_path.open(encoding='utf-8') as stream:
                    metrics = json.load(stream)
                metrics.update(metadata)
                runs.append(metrics)
                continue
            command = [
                str(script),
                bag['path'],
                str(run_dir),
                str(args.domain_id),
                str(segment['start_offset_sec']),
                str(segment['duration_sec']),
            ]
            print(' '.join(command), flush=True)
            if args.dry_run:
                continue
            result = subprocess.run(command, env=environment, check=False)
            if result.returncode or not metrics_path.is_file():
                failures.append({
                    **metadata,
                    'return_code': result.returncode,
                })
                _write_summary(output, runs, failures, configuration)
                if not args.continue_on_error:
                    return 1
                continue
            with metrics_path.open(encoding='utf-8') as stream:
                metrics = json.load(stream)
            metrics.update(metadata)
            runs.append(metrics)
            _write_summary(output, runs, failures, configuration)
    _write_summary(output, runs, failures, configuration)
    return int(bool(failures))


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument(
        '--split',
        choices=['all', 'tune', 'holdout'],
        default='all',
    )
    parser.add_argument('--bag-name', action='append', default=[])
    parser.add_argument('--max-runs', type=int, default=0)
    parser.add_argument('--domain-id', type=int, default=88)
    parser.add_argument('--rate', type=float, default=0.25)
    parser.add_argument('--target-fps', type=float, default=8.0)
    parser.add_argument('--imgsz', type=int, default=960)
    parser.add_argument('--confidence-threshold', type=float, default=0.35)
    parser.add_argument('--iou-threshold', type=float, default=0.45)
    parser.add_argument(
        '--model',
        default=str(Path.home() / 'models/yolo/yolo11n_960.engine'),
    )
    parser.add_argument(
        '--reid-model',
        default=str(Path.home() / 'models/yolo/yolo26n-cls.pt'),
    )
    parser.add_argument('--tracker-config', default='')
    parser.add_argument('--script', default='')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--continue-on-error', action='store_true')
    return parser.parse_args(argv)


def main() -> None:
    """CLI entry point."""
    raise SystemExit(run(parse_args()))


if __name__ == '__main__':
    main()
