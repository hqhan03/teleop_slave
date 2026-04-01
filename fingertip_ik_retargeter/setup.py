from setuptools import setup
import os
from glob import glob

package_name = 'fingertip_ik_retargeter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['data/*.urdf']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fingertip_ik_node = fingertip_ik_retargeter.fingertip_ik_node:main',
            'offline_ik_test = fingertip_ik_retargeter.offline_ik_test:main',
            'calibrate_ik_frame = fingertip_ik_retargeter.calibrate_ik_frame:main',
            'calibrate_ik_multipose = fingertip_ik_retargeter.calibrate_ik_multipose:main',
            'generate_kapandji_poses = fingertip_ik_retargeter.generate_kapandji_poses:main',
        ],
    },
)
