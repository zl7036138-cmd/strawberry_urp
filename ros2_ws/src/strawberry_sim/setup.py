"""Setuptools entry point for the strawberry_sim ROS 2 package."""

from __future__ import annotations

import os
from setuptools import find_packages, setup


PACKAGE_NAME = "strawberry_sim"


def share_data_files(*directories: str) -> list[tuple[str, list[str]]]:
    """Preserve asset directory layout under share/strawberry_sim."""

    result: list[tuple[str, list[str]]] = []
    for directory in directories:
        for current, dirnames, filenames in os.walk(directory):
            # Importing a launch file can create transient Python bytecode in
            # the source tree.  Never let those cache files enter setuptools'
            # persistent manifest, where their later disappearance breaks an
            # otherwise valid incremental ROS build.
            dirnames[:] = [
                dirname for dirname in dirnames if dirname != "__pycache__"
            ]
            files = [
                os.path.join(current, filename)
                for filename in filenames
                if not filename.endswith((".pyc", ".pyo"))
            ]
            if files:
                result.append((os.path.join("share", PACKAGE_NAME, current), files))
    return result


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        *share_data_files("launch", "config", "worlds", "models", "urdf"),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Strawberry URP Team",
    maintainer_email="strawberry-urp@example.invalid",
    description=(
        "Gazebo Harmonic world, RGB-D bridge, ground truth, and fail-safe "
        "fruit attachment services."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "attachment_manager = strawberry_sim.attachment_manager:main",
            "contact_monitor = strawberry_sim.contact_monitor:main",
            "ground_truth_publisher = strawberry_sim.ground_truth_publisher:main",
            "generalized_development_capture = strawberry_sim.generalized_capture:main",
            "generate_generalized_scene = strawberry_sim.generalized_scene:main",
            "runtime_health_check = strawberry_sim.runtime_health:main",
            "scene_condition_probe = strawberry_sim.scene_condition_probe:main",
            "synthetic_capture = strawberry_sim.synthetic_capture:main",
        ],
    },
)
