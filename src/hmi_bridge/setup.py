import os

from setuptools import find_packages, setup

package_name = 'hmi_bridge'


def data_files_for(src_dir, dest_prefix):
    """src_dir 아래 모든 파일을 share/<package>/<dest_prefix>/<상대경로>에 설치되게
    (dir, [files]) 튜플 목록을 만든다 (템플릿/정적파일처럼 디렉토리 구조를 유지해야
    하는 리소스용)."""
    entries = []
    for root, _dirs, files in os.walk(src_dir):
        if not files:
            continue
        rel = os.path.relpath(root, src_dir)
        dest = os.path.join('share', package_name, dest_prefix, rel) if rel != '.' \
            else os.path.join('share', package_name, dest_prefix)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + data_files_for('templates', 'templates') + data_files_for('static', 'static'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soo',
    maintainer_email='poi3824@gmail.com',
    description='Flask 기반 HMI - ROS2 시스템(월드맵/장애물/작업 상태)을 관제만 하는 웹 UI.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hmi_bridge_server = hmi_bridge.app:main',
        ],
    },
)
