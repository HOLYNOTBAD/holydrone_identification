"""验证同航次按完整双脉冲留出：通道/时长覆盖、时序隔离和不足数据拒绝。"""
import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np
from brct1order_process import split_pulse_groups
from brct1order_regression import CHANNELS, read_csv, segments


class PulseGroupSplitTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root/'all.csv'
        self.train = self.root/'train.csv'
        self.validation = self.root/'validation.csv'

    def acquisition(self, repetitions=3, widths=(.35,)):
        rows = []
        segment = 0
        for repeat in range(repetitions):
            for width in widths:
                for axis, channel in enumerate(CHANNELS):
                    for relative in np.arange(0, 1.5, .02):
                        pulse = 1 if .2 <= relative < .2+width else -1 if .2+width <= relative < .2+2*width else 0
                        action = [0., 0., 0., .3]
                        action[axis] += pulse*(.015 if axis == 3 else .2)
                        rows.append(dict(t=segment*2+relative, cmd_wx=action[0], cmd_wy=action[1],
                                         cmd_wz=action[2], cmd_thrust=action[3], wx=0., wy=0., wz=0., fz=9.81,
                                         segment=segment, channel=channel, valid=1, phase='pulse',
                                         sequence_complete=0, pulse_hold_s=width))
                    segment += 1
        rows[-1]['sequence_complete'] = 1
        with self.source.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0])
            writer.writeheader(); writer.writerows(rows)

    def test_three_rounds_hold_out_last_round_without_shared_samples(self):
        self.acquisition()
        split = split_pulse_groups(self.source, self.train, self.validation)
        self.assertEqual([g['segment'] for g in split['training_groups']], list(range(8)))
        self.assertEqual([g['segment'] for g in split['validation_groups']], list(range(8, 12)))
        train, validation = read_csv(self.train), read_csv(self.validation)
        self.assertFalse(set(train['t']) & set(validation['t']))
        self.assertEqual(len(train)+len(validation), len(read_csv(self.source)))
        for channel in range(4):
            self.assertEqual(len(segments(train, channel)), 2)
            self.assertEqual(len(segments(validation, channel)), 1)

    def test_second_order_preserves_every_channel_and_duration(self):
        self.acquisition(repetitions=2, widths=(.2, .35, .5))
        split = split_pulse_groups(self.source, self.train, self.validation)
        self.assertEqual(len(split['training_groups']), 12)
        self.assertEqual(len(split['validation_groups']), 12)
        for key in ('training_groups', 'validation_groups'):
            self.assertEqual({(g['channel'], g['pulse_hold_s']) for g in split[key]},
                             {(channel, width) for channel in CHANNELS for width in (.2, .35, .5)})
        self.assertLess(max(g['end_s'] for g in split['training_groups']),
                        min(g['start_s'] for g in split['validation_groups']))

    def test_one_round_and_invalid_ratio_are_rejected(self):
        self.acquisition(repetitions=1)
        with self.assertRaisesRegex(ValueError, 'at least two complete'):
            split_pulse_groups(self.source, self.train, self.validation)
        for ratio in (0, 1, float('nan')):
            with self.assertRaises(ValueError):
                split_pulse_groups(self.source, self.train, self.validation, ratio)


if __name__ == '__main__':
    unittest.main()
