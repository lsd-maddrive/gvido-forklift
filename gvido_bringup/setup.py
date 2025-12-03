from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gvido_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'gvido_bringup/scripts'), glob(os.path.join('gvido_bringup/scripts', '*.py'))),    
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
            'gvido_can_bringup = gvido_bringup.scripts.gvido_can_bringup:main',
        ],
    },
)
