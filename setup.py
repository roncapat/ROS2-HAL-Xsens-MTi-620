import os
from glob import glob
from setuptools import setup

package_name = 'hal_xsens_mti_620'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roncapat',
    maintainer_email='roncapat@gmail.com',
    description='HAL for Xsens MTi 620 VRU',
    license='GPLv3',
    entry_points={
        'console_scripts': [
        "hal_xsens_mti_620 = hal_xsens_mti_620.hal_xsens_mti_620:main",
        ],
    },
)
