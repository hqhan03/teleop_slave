from glob import glob
import os

from setuptools import setup


package_name = 'keyvector_retargeter'


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
    install_requires=['setuptools', 'numpy', 'PyYAML', 'scipy'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'keyvector_retarget_node = keyvector_retargeter.keyvector_retarget_node:main',
            'calibrate_keyvector = keyvector_retargeter.calibrate_keyvector:main',
        ],
    },
)
