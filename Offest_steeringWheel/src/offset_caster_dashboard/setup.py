from setuptools import find_packages, setup

package_name = "offset_caster_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Offset Caster Maintainer",
    maintainer_email="maintainer@example.com",
    description="Standalone Qt control and telemetry dashboard for the offset-caster chassis.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "offset_caster_dashboard = offset_caster_dashboard.dashboard:main",
            "offset_caster_monitor = offset_caster_dashboard.monitor:main",
        ],
    },
)
