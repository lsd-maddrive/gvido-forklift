from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gvido_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'calibration'), glob(os.path.join('config', 'calibration', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch/drivers'), glob(os.path.join('launch/drivers', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'gvido_vision/scripts'), glob(os.path.join('gvido_vision/scripts', '*.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gvido-boss',
    maintainer_email='gvido-boss@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stereo_filter = gvido_vision.scripts.stereo_filter:main',
            'stereo_tag_maper = gvido_vision.scripts.stereo_tag_maper:main',
            'apriltag_detector = gvido_vision.scripts.apriltag_detector:main',
        ],
    },
)
