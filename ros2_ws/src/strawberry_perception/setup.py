from glob import glob
from setuptools import find_packages, setup


package_name = "strawberry_perception"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Strawberry URP team",
    maintainer_email="strawberry-urp@example.com",
    description="Deterministic strawberry maturity detection for the URP simulator.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = strawberry_perception.perception_node:main",
            "shadow_diagnostic = strawberry_perception.shadow_diagnostic:main",
            "sim_perception_gate = strawberry_perception.sim_perception_gate:main",
            "shadow_window_probe = strawberry_perception.shadow_window_probe:main",
        ],
    },
)
