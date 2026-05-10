import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'scan_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuxuan',
    maintainer_email='yuxuan@todo.todo',
    description='Simple laser scan based cmd_vel safety filter.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simple_scan_safety = scan_safety.simple_scan_safety:main',
        ],
    },
)
