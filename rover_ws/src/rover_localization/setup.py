from glob import glob
from setuptools import find_packages, setup

setup(
    name='rover_localization', version='0.0.1', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rover_localization']),
        ('share/rover_localization', ['package.xml', 'README.md']),
        *[(f'share/rover_localization/{d}', glob(f'{d}/*{extension}'))
          for d, extension in (('launch', '.launch.py'), ('config', '.yaml'), ('rviz', '.rviz'))],
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Space Bulls', maintainer_email='june.bang.h@gmail.com',
    description='FAST-LIO simulation diagnostics', license='Apache-2.0',
    entry_points={'console_scripts': [
        'odometry_comparator = rover_localization.odometry_comparator:main',
        'cmd_vel_watchdog = rover_localization.cmd_vel_watchdog:main',
    ]},
)
