from glob import glob
from setuptools import setup
package_name="ece3162_bringup"
setup(name=package_name, version="0.2.0", packages=[package_name],
 data_files=[("share/ament_index/resource_index/packages",["resource/"+package_name]),
 ("share/"+package_name,["package.xml"]),("share/"+package_name+"/launch",glob("launch/*.launch.py")),
 ("share/"+package_name+"/config",glob("config/*"))], install_requires=["setuptools"], zip_safe=True,
 tests_require=["pytest"], maintainer="ECE3162 maintainers",maintainer_email="maintainer@example.com",description="ECE3162 bringup",license="MIT")
