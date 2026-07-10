from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soo',
    maintainer_email='poi3824@gmail.com',
    description='RealSense 컬러/depth 이미지에서 YOLO로 물체를 찾아 카메라 좌표계 3D 위치를 반환하는 노드',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detection_node = object_detection.detection:main',
            'depth_probe = object_detection.depth_probe:main',
            'angle_probe = object_detection.angle_probe:main',
            'yolo_probe = object_detection.yolo_probe:main',
        ],
    },
)
