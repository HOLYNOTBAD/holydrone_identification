"""用合成 ULog 话题验证提取、整组留出与启动模型参数恢复。"""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]

def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/'escmotor'/f'{name}.py')
    result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
    return result


class Pipeline(unittest.TestCase):
    def test_synthetic_extraction_and_identification(self):
        process = module('escmotor_expdataprocess')
        identify = module('escmotor_identification')
        command = {key: [] for key in ['timestamp', 'function', 'value', 'action', 'timeout_ms']}
        events = []
        t = np.arange(.1, 10., .005)
        omega = np.zeros(len(t))
        for i, begin in enumerate([1., 4., 7.]):
            u = .1 if i == 0 else .2
            for stamp in np.arange(begin, begin+1., .2):
                for key, value in zip(command, [stamp*1e6, 101, u, 1, 600]): command[key].append(value)
            for key, value in zip(command, [(begin+1)*1e6, 101, np.nan, 0, 3000]): command[key].append(value)
            events.extend([dict(event='pulse_start', command=u, phase='precheck' if i == 0 else 'experiment',
                                last_esc_time_us=(begin-.01)*1e6, target_voltage_v=12.), dict(event='pulse_complete')])
            mask = (t >= begin) & (t < begin+1)
            omega[mask] = identify.predict(t[mask]-begin, 0., 800., .04, .02)
        topics = dict(actuator_test={k: np.array(v) for k,v in command.items()},
                      esc_status={'esc[0].timestamp': t*1e6, 'esc[0].esc_rpm': omega*60/(2*np.pi),
                                  'esc[0].esc_voltage': t*0+12., 'esc[0].esc_current': t*0+1.})
        topics['battery_status'] = dict(id=np.ones(len(t)), timestamp=t*1e6, voltage_v=t*0+12.)
        fake = types.SimpleNamespace(ULog=lambda path: types.SimpleNamespace(
            dropouts=[], get_dataset=lambda name, instance: types.SimpleNamespace(data=topics[name])))
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); session = root/'session'; session.mkdir()
            c = yaml.safe_load((ROOT/'config/escmotor/escmotor.yaml').read_text())
            c['voltage_source'] = 'battery'
            c['battery_id'] = 0
            (session/'config.yaml').write_text(yaml.safe_dump(c))
            (session/'events.jsonl').write_text('\n'.join(map(json.dumps, events)))
            ulg = root/'test.ulg'; ulg.write_bytes(b'synthetic ULog topics injected by test')
            dataset = root/'dataset'; report = root/'report.yaml'
            with patch.dict(sys.modules, {'pyulog': fake}), patch.object(sys, 'argv', ['process','--ulog',str(ulg),'--session',str(session),'--output',str(dataset)]):
                process.main()
            meta = yaml.safe_load((dataset/'dataset.yaml').read_text())
            self.assertEqual([g['split'] for g in meta['groups']], ['train','validation'])
            with patch.object(sys, 'argv', ['identify','--dataset',str(dataset),'--output',str(report)]):
                identify.main()
            row = yaml.safe_load(report.read_text())['conditions'][0]
            self.assertTrue(row['passed'])
            self.assertAlmostEqual(row['startup_tau_s'], .04, places=3)
            self.assertAlmostEqual(row['startup_delay_s'], .02, places=3)

    def test_requires_repeated_groups(self):
        process = module('escmotor_expdataprocess')
        with self.assertRaises(ValueError):
            process.split_groups([dict(target_voltage_v=12., u=.2)])

    def test_expired_command_splits_pulses(self):
        process = module('escmotor_expdataprocess')
        result = process.intervals(dict(timestamp=[1000000, 2000000, 2100000],
            function=[101]*3, value=[.2,.2,np.nan], action=[1,1,0], timeout_ms=[100,100,0]),101)
        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(result[0]['end'],1.1)

if __name__ == '__main__': unittest.main()
