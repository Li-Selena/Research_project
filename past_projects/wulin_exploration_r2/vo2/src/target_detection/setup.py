from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'target_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vo2 contributors',
    maintainer_email='maintainers@example.com',
    description='Camera capture and YOLO detection node for vo2.',
    license='LicenseRef-Unknown',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_detection = target_detection.target_detection:main',
        ],
    },
)
