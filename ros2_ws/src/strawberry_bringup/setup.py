from glob import glob

from setuptools import find_packages, setup


package_name = "strawberry_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Strawberry URP",
    maintainer_email="urp@example.invalid",
    description="Deterministic trial orchestration.",
    license="AGPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_selector = strawberry_bringup.target_selector:main",
            "harvest_orchestrator = strawberry_bringup.harvest_orchestrator:main",
            "generalized_development_probe = strawberry_bringup.development_probe:main",
            "generalized_development_score = strawberry_bringup.development_gate:main",
            "generalized_feasibility_probe = strawberry_bringup.feasibility_probe:main",
            "generalized_truth_isolation_audit = strawberry_bringup.truth_isolation_audit:main",
            "dual_observation_repeat_summary = strawberry_bringup.observation_repeat:main",
            "dual_observation_summary = strawberry_bringup.observation_sequence:main",
            "dual_observation_selector = strawberry_bringup.observation_selection:main",
            "oracle_target_provider = strawberry_bringup.oracle_target_provider:main",
            "trial_orchestrator = strawberry_bringup.orchestrator:main",
        ],
    },
)
