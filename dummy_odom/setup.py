from setuptools import setup

package_name = 'dummy_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
      ('share/ament_index/resource_index/packages',
       ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            # this makes a binary called "dummy_odom_node"
            'dummy_odom_node = dummy_odom.dummy_odom_node:main',
        ],
    },
)

