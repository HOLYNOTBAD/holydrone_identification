#!/usr/bin/env python3
"""从一次 PX4 ULog 按完整脉冲组划分训练/验证，拟合 BRCT 二阶响应。"""
from pathlib import Path
import sys

# 支持 Python -I 隔离模式下导入同目录回归模块和数据处理入口。
sys.path.insert(0, str(Path(__file__).resolve().parent))
from brct2order_regression import identify_response
from brct1order_process import run_identification


def main():
    run_identification(identify_response, 'second-order')


if __name__ == '__main__':
    main()
