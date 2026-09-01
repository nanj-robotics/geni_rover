from setuptools import find_packages, setup

package_name = 'gps_ntrip_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wt',
    maintainer_email='wt@todo.todo',
    description='GPS NTRIP bridge node (serial GNSS <-> NTRIP caster) - ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_ntrip_node = gps_ntrip_py.gps_ntrip_node:main',
        ],
    },
)
