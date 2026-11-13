"""统一采集配置与会话文件验证；不连接 ROS 或飞控。"""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


class FlightCollectionTest(unittest.TestCase):
    def test_launch_and_config(self):
        config = yaml.safe_load((ROOT / 'config/excitation/pulse_medium.yaml').read_text())
        self.assertEqual(config['pulse_hold'], .2)
        self.assertEqual(config['repetitions'], 3)
        self.assertNotIn('pulse_durations_s', config)
        launch = ET.parse(ROOT / 'launch/excitation/flight_collect.launch').getroot()
        args = {a.attrib['name']: a.attrib.get('value') for a in launch.findall('include/arg')}
        self.assertEqual(args['exciter'], 'flight_excite')
        self.assertTrue(launch.find("node[@type='flight_session.py']") is not None)

    def test_session_preserves_config_completion_and_existing_files(self):
        rospy = types.SimpleNamespace(
            Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 12.5)),
            Subscriber=lambda *a, **k: None, on_shutdown=lambda callback: None)
        modules = {'rospy': rospy, 'std_msgs': types.ModuleType('std_msgs'),
                   'std_msgs.msg': types.SimpleNamespace(String=object)}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('flight_session_test', ROOT / 'scripts/flight/flight_session.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'session'
            session = module.Session(out, ROOT / 'config/excitation/pulse_medium.yaml')
            for complete in (False, True, True):
                session.status(types.SimpleNamespace(data=json.dumps({'sequence_complete': complete})))
            session.close()
            records = [json.loads(line) for line in (out / 'events.jsonl').read_text().splitlines()]
            self.assertEqual(sum(r['event'] == 'experiment_complete' for r in records), 1)
            self.assertTrue(records[-1]['data']['experiment_complete'])
            self.assertEqual((out / 'config.yaml').read_bytes(), (ROOT / 'config/excitation/pulse_medium.yaml').read_bytes())
            original = (out / 'events.jsonl').read_bytes()
            with self.assertRaises(FileExistsError):
                module.Session(out, ROOT / 'config/excitation/pulse_medium.yaml')
            self.assertEqual((out / 'events.jsonl').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
