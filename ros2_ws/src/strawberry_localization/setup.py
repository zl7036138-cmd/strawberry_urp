from glob import glob
from setuptools import find_packages, setup


package_name = "strawberry_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Strawberry URP",
    maintainer_email="urp@example.invalid",
    description="Robust RGB-D localization for strawberry targets.",
    license="AGPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization_node = strawberry_localization.node:main",
            "localization_gate = strawberry_localization.localization_gate:main",
        ],
    },
)
