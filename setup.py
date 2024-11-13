from setuptools import setup
from glob import glob

package_name = 'go2_control_interface'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/' + package_name, ['package.xml']),
     ('lib/' + package_name, glob("scripts/*.py")),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='EtienneAr',
 maintainer_email='etienne.arlaud@inria.fr',
 description='Wrapper for sending joint command to the go2 robot, with safeties and init procedure.',
 license='BSD-3-Clause',
#  tests_require=['pytest'],
 entry_points={},
)