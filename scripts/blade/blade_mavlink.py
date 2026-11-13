"""为不含 PX4 ESC_STATUS 的 pymavlink common 方言补充消息解码。"""
import importlib.util
from pathlib import Path
import tempfile


def enable_esc_status(mavutil):
    mavutil.set_dialect('common')
    if 291 in mavutil.mavlink.mavlink_map:
        return
    from pymavlink.generator import mavgen
    definition = Path(__file__).with_name('blade_messages.xml')
    with tempfile.TemporaryDirectory(prefix='blade-mavlink-') as directory:
        output = Path(directory)/'esc_messages.py'
        options = mavgen.Opts(str(output), wire_protocol='2.0', language='Python3', validate=False)
        if not mavgen.mavgen(options, [str(definition)]):
            raise RuntimeError('无法生成 PX4 ESC_STATUS 消息定义')
        spec = importlib.util.spec_from_file_location('blade_messages', output)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    mavutil.mavlink.mavlink_map[291] = module.MAVLink_esc_status_message
