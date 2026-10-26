from setuptools import find_packages, setup
package_name = "ece3162_mission"
setup(name=package_name, version="0.2.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"], zip_safe=True,
      maintainer="ECE3162 maintainers", maintainer_email="maintainer@example.com",
      description="Low-rate mission coordination and safety state machine", license="MIT",
      entry_points={"console_scripts": ["mission_manager = ece3162_mission.manager:main"]})
