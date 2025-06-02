from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'farol_bringup'     # package name
module_name = 'farol_bringup_node' # python file name where node is

def package_files(directory):
  paths = []
  for (path, _, filenames) in os.walk(directory):
    for filename in filenames:
      full_path = os.path.join(path, filename)
      install_path = os.path.join('share', package_name, os.path.relpath(path, '.'))
      paths.append((install_path, [full_path]))
  return paths

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(where='src'),  # search for packages inside 'src'
  package_dir={'': 'src'},              # declare that root packages are under 'src'
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
  ] + package_files('config_default'), # install config files as well
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='Eduardo Cunha',
  maintainer_email='eduardo.m.a.cunha@tecnico.ulisboa.pt',
  description='FAROL_BRINGUP: Bringup to launch all packages and handle process management',
  license='MIT',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      # makes sure node in package can be run with 'ros2 run'
      module_name + ' = ' + package_name + '.' + module_name + ':main',
    ],
  },
)
