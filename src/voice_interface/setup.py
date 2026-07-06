from setuptools import find_packages, setup
import glob

package_name = 'voice_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # glob.glob('resource/*')는 점(.)으로 시작하는 파일(.env)을 잡지 못해서
        # 따로 명시해야 한다 (pick_and_place_voice/setup.py에서도 동일하게 처리했음).
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/resource', glob.glob('resource/.env')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soo',
    maintainer_email='poi3824@gmail.com',
    description='음성 명령을 받아 STT+LLM으로 pick&place 키워드를 추출하는 robot_get_keyword_node',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'get_keyword_node = voice_interface.robot_get_keyword_node:main',
        ],
    },
)
