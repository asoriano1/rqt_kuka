# setup.py
from setuptools import setup

setup(
    name='rqt_kuka_plugin',
    version='0.1.0',
    packages=['rqt_kuka_plugin'],
    package_dir={'rqt_kuka_plugin': 'scripts/rqt_kuka_plugin'},
    package_data={'rqt_kuka_plugin': ['resource/*.ui']},
    install_requires=['setuptools'],
    entry_points={
        'qt_gui.plugin': [
            'rqt_kuka_plugin = rqt_kuka_plugin.main_plugin:RqtKukaPlugin',
        ],
    },
)

