from setuptools import find_packages, setup


package_name = 'robot_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'CONTROL.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vo2 contributors',
    maintainer_email='maintainers@example.com',
    description='Bidirectional control and feedback bridge for the STM32 robot.',
    license='LicenseRef-Unknown',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot_serial_bridge = robot_serial_bridge.serial_bridge:main',
        ],
    },
)
