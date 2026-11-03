#!/usr/bin/env python3
"""从两次独立 PX4 ULog 提取完整脉冲，拟合并验证 BRCT 二阶响应。
将 FRD 日志转换为 FLU，按零阶保持对齐指令；输出报告，不修改控制参数。"""
import argparse
import hashlib
from pathlib import Path
import tempfile
import sys

# Support the package source tree inside the analysis Conda environment.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import yaml

from response2 import identify_response


# 源码和 Catkin 安装入口均可复用同包的一阶日志提取器。
sys.path.insert(0, str(Path(__file__).resolve().parent))
from brct1order_process import ulog_to_csv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True,
                        help='Training .ulg copied from the PX4 SD card')
    parser.add_argument('--validation', type=Path, required=True,
                        help='Independent validation .ulg copied from the PX4 SD card')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; use a new path')
    for path in (args.train, args.validation):
        if path.suffix.lower() != '.ulg' or not path.is_file():
            parser.error(str(path)+' must be an existing PX4 .ulg file')
    with tempfile.TemporaryDirectory() as directory:
        training_csv = Path(directory) / 'training.csv'
        validation_csv = Path(directory) / 'validation.csv'
        ulog_to_csv(args.train, training_csv)
        ulog_to_csv(args.validation, validation_csv)
        report = identify_response(training_csv, validation_csv)
    report['training_sha256'] = hashlib.sha256(args.train.read_bytes()).hexdigest()
    report['validation_sha256'] = hashlib.sha256(args.validation.read_bytes()).hexdigest()
    report['source_logs'] = [str(args.train.resolve()), str(args.validation.resolve())]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print('BRCT second-order report:', args.output, 'passed:', report['passed'])


if __name__ == '__main__':
    main()
