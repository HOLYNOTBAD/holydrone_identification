"""将四通道的拟合数据、验证数据和一至三阶模型预测画在同一张图中。"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from brct1order_regression import CHANNELS, predict as predict_first, read_csv, segments
from brct2order_regression import predict as predict_second
from brct3order_regression import predict as predict_third, PARAMETER_KEYS as THIRD_KEYS


def plot_response(train_path, validation_path, report, output):
    """复用辨识的片段筛选和预测算法；独立脉冲之间留空，不跨片段连线。"""
    training, validation = read_csv(train_path), read_csv(validation_path)
    second = report.get('model') == 'second_order_with_delay'
    third = report.get('model') == 'third_order_with_delay'
    predict = predict_third if third else predict_second if second else predict_first
    keys = THIRD_KEYS if third else (('natural_frequency_rad_s', 'damping_ratio', 'delay_s', 'gain', 'bias')
            if second else ('tau_s', 'delay_s', 'gain', 'bias'))
    results = {row['channel']: row for row in report['channels']}
    fig, axes = plt.subplots(4, 1, figsize=(24, 16), constrained_layout=True)
    for index, (name, ax) in enumerate(zip(CHANNELS, axes.flat)):
        row = results[name]
        params = [row[key] for key in keys]
        command_ax = ax.twinx() if name == 'thrust' else ax
        if name == 'thrust':
            command_ax.set_ylabel('Normalized thrust command', color='0.45')
        else:
            ax.axhline(0, color='0.6', linewidth=.7)
        offset = 0.0
        for data, label, color in ((training, 'Training data', '#0072B2'),
                                   (validation, 'Validation data', '#D55E00')):
            for pulse_index, (t, u, y) in enumerate(segments(data, index)):
                command_ax.step(t + offset, u, where='post', color='0.5',
                                linewidth=1, alpha=.65,
                                label='Input command' if label == 'Training data' and pulse_index == 0 else None)
                ax.plot(t + offset, y, color=color, linewidth=1.6,
                        label=label if pulse_index == 0 else None)
                ax.plot(t + offset, predict(t, u, y[0], *params),
                        color='#222222', linestyle='--', linewidth=1.2,
                        label='Model prediction' if label == 'Training data' and pulse_index == 0 else None)
                offset += float(t[-1]) + 0.12
            if label == 'Training data':
                ax.axvline(offset - 0.06, color='0.65', linestyle=':', linewidth=1)
        dynamics = (f'wn={row["natural_frequency_rad_s"]:.2f} rad/s, zeta={row["damping_ratio"]:.3f}'
                    if second or third else f'tau={row["tau_s"]*1000:.1f} ms')
        if third:
            dynamics = f'tau={row["tau_s"]*1000:.1f} ms, '+dynamics
        ax.set_title(f'{name.upper()}  |  {dynamics}, delay={row["delay_s"]*1000:.1f} ms\n'
                     f'gain={row["gain"]:.4g}, bias={row["bias"]:.4g}  |  '
                     f'RMSE train={row["training"]["rmse"]:.4f}, val={row["validation"]["rmse"]:.4f}',
                     fontsize=10)
        ax.set_xlabel('Concatenated pulse time (s; gaps omit recovery)')
        ax.set_ylabel('Specific force (m/s²)' if name == 'thrust' else 'Angular rate (rad/s)')
        ax.grid(alpha=0.2)
        handles, labels = ax.get_legend_handles_labels()
        if command_ax is not ax:
            extra_handles, extra_labels = command_ax.get_legend_handles_labels()
            handles += extra_handles
            labels += extra_labels
        ax.legend(handles, labels, fontsize=8, loc='best')
    order = 'Third-order' if third else 'Second-order' if second else 'First-order'
    initial = 'Each pulse prediction starts at its first measured output'
    if third:
        initial += ' with zero first and second derivatives'
    elif second:
        initial += ' with zero initial derivative'
    fig.suptitle(f'{order} response fit — training and held-out validation\n{initial}\n'
                 'Gaps omit position recovery; trace endpoints are not steady-state measurements', fontsize=13)
    try:
        fig.savefig(output, dpi=180)
    finally:
        plt.close(fig)
