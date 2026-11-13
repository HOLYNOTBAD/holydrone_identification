# Blade 四组拟合

打开 `index.html` 离线查看四组系数、叠加曲线和逐点残差。今日组固定对应 `bench_20261113_113812`。

在仓库根目录重建：

```bash
conda run --no-capture-output -n holydrone_autotune python scripts/blade/blade_fit_compare.py
```

脚本先重建 dataset。每组子目录保存 report.json、fit_points.csv 及 SVG/PNG/PDF 图。联合拟合使用所选三个实验的全部有效点，每点等权，不是每个实验等权。

模型 F=k_f×omega²，F 用 N，omega 用 rad/s。图中只绘制各组已测 RPM 区间；全部误差为样本内误差。厂家缺 RPM 的一行排除。不同来源硬件一致性未确认，联合系数是三组数据的共同近似。
