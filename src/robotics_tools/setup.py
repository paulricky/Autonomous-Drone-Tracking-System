"""ament entry point for the existing robotics_tools Python algorithm package."""
from setuptools import find_packages, setup

setup(name="robotics_tools", version="0.2.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/robotics_tools"]),
                  ("share/robotics_tools", ["package.xml"])],
      install_requires=["setuptools", "numpy", "PyYAML"], zip_safe=False,
      tests_require=["pytest"],
      maintainer="Autonomous Drone Navigation maintainers", maintainer_email="maintainer@example.com",
      description="Python hand-goal, offline geometry, and simulation tools", license="MIT")
