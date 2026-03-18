from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obj_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'images'), glob(os.path.join('images', '*.png'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.sdf'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arz-1024',
    maintainer_email='bineeshajabi98@gmail.com',
    description='Object Detection Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "cam_open = obj_detect.open_cam:main",
            "dist_hsv = obj_detect.hsv_obj_dist_single:main",
            "dist_multiple_hsv = obj_detect.hsv_obj_multiple:main",
        ],
    },
)
