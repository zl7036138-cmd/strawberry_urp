from setuptools import find_packages, setup


package_name = "strawberry_benchmark"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Strawberry URP Team",
    maintainer_email="strawberry-urp@example.invalid",
    description=(
        "Dependency-light benchmark scenarios, trial logs, metrics, and "
        "acceptance gates for Strawberry URP."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "strawberry-benchmark = strawberry_benchmark.cli:main",
            "generalized-harvest-acceptance = strawberry_benchmark.generalized_acceptance:main",
            "claim-generalized-harvest = strawberry_benchmark.generalized_claim:main",
            "generalized-detector-eval = strawberry_benchmark.generalized_detector_eval:main",
            "materialize-generalized-harvest = strawberry_benchmark.materialize_generalized:main",
            "strawberry-evidence-contract = strawberry_benchmark.run_identity:main",
        ],
    },
)
