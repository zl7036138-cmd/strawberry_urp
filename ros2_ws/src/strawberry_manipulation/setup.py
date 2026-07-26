import os

from setuptools import find_packages, setup


package_name = "strawberry_manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                os.path.join("config", filename)
                for filename in os.listdir("config")
                if os.path.isfile(os.path.join("config", filename))
            ],
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Strawberry URP",
    maintainer_email="urp@example.invalid",
    description="MoveIt-backed Panda pick-and-place action server.",
    license="AGPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "handoff_shadow_repeat_summary = strawberry_manipulation.handoff_repeat:main",
            "handoff_shadow_probe = strawberry_manipulation.handoff_shadow:main",
            "pregrasp_planning_shadow = strawberry_manipulation.pregrasp_shadow:main",
            "pregrasp_shadow_repeat_summary = strawberry_manipulation.pregrasp_repeat:main",
            "pick_and_place_server = strawberry_manipulation.action_server:main",
        ],
    },
)
