from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('drone_tracker/models/*.pt')),
        ('share/' + package_name + '/models', glob('drone_tracker/models/*.onnx')),
        ('share/' + package_name + '/models', glob('drone_tracker/models/*.engine')),
        (os.path.join('share',package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usrl1234',
    maintainer_email='usrl1234@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telescope = drone_tracker.camera:main',
            'detect = drone_tracker.detector:main',
            'track_cv = drone_tracker.tracker_cv:main',
            'track_sort = drone_tracker.tracker_sort:main',
            'track_dsort = drone_tracker.tracker_dsort:main',
            'movement = drone_tracker.camera_movement:main',
            'viewer = drone_tracker.viewer:main',
            'vid_playback = drone_tracker.vid_player:main',
            'data = drone_tracker.data_gather:main',
            'detect_kcf = drone_tracker.detector_kcf:main',
        ],
    },
)
