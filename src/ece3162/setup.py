"""ament entry point for the existing ece3162 Python algorithm package."""
from setuptools import find_packages, setup

setup(name="ece3162_algorithms", version="0.2.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/ece3162_algorithms"]),
                  ("share/ece3162_algorithms", ["package.xml"])],
      install_requires=["setuptools", "numpy", "scipy", "PyYAML"], zip_safe=False,
      tests_require=["pytest"],
      maintainer="ECE3162 maintainers", maintainer_email="maintainer@example.com",
      description="Validated ECE3162 Python algorithm library", license="MIT")
