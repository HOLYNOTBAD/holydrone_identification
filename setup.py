"""向 Catkin 注册 scripts 下的独立 Python 分析模块。"""
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    py_modules=['brct1order_regression', 'brct2order_regression'],
    # 空键供 setuptools 定位模块，模块名键供 Catkin 生成 devel 转发入口。
    package_dir={'': 'scripts', 'brct1order_regression': 'scripts', 'brct2order_regression': 'scripts'},
))
