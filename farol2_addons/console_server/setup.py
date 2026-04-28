from setuptools import find_packages, setup

package_name = 'console_server'     # package name
module_name = 'console_server'      # python file name where node is

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(where='src'),  # search for packages inside 'src'
  package_dir={'': 'src'},              # declare that root packages are under 'src'
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='Eduardo Cunha',
  maintainer_email='eduardo.m.a.cunha@tecnico.ulisboa.pt',
  description='console_server',
  license='MIT',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      # makes sure node in package can be run with 'ros2 run'
      package_name + ' = ' + package_name + '.' + module_name + ':main',
    ],
  },
)
