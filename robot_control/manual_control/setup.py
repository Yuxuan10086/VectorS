import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'manual_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'web'), glob(os.path.join('web', '**/*'), recursive=True)),
    ],
    install_requires=['setuptools', 'fastapi>=0.110.0', 'uvicorn[standard]>=0.30.0'],
    zip_safe=True,
    maintainer='yuxuan',
    maintainer_email='yuxuan@todo.todo',
    description='Web-based manual control panel (FastAPI + WebSocket) for chassis and SCARA arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_control_panel = manual_control.web_control_panel:main',
        ],
    },
)
