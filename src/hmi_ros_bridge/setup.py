from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hmi_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'python-socketio[client]', 'python-dotenv'],
    zip_safe=True,
    maintainer='soo',
    maintainer_email='poi3824@gmail.com',
    description='ROS 2 Topic/Service/Action과 hmi/backend(Flask-SocketIO) 사이의 전용 Bridge Node',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hmi_ros_bridge_server = hmi_ros_bridge.main:main',
            'hmi_vision_stream = hmi_ros_bridge.vision_stream_node:main',
        ],
    },
)
