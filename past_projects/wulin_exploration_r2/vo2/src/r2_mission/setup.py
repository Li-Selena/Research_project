from setuptools import find_packages, setup


package_name = 'r2_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'MISSION.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vo2 contributors',
    maintainer_email='maintainers@example.com',
    description='Nav2, terrain traversal and object picking mission server.',
    license='LicenseRef-Unknown',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mission_server = r2_mission.mission_server:main',
        ],
    },
)
