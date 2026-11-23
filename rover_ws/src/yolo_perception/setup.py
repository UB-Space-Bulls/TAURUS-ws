from glob import glob

from setuptools import find_packages, setup

package_name = 'yolo_perception'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test', 'training']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Nguyen',
    maintainer_email='Thnguyen7807@gmail.com',
    description=('YOLO object detection for the Space Bulls rover '
                 '(terrain, rock hammer, keyboard keys)'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_yolo_node = yolo_perception.zed_yolo_node:main',
            'mock_zed_node = yolo_perception.mock_zed_node:main',
            'offline_tester = yolo_perception.offline_tester:main',
        ],
    },
)
