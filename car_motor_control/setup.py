from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 config 目录下的所有 .yaml 文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 安装 launch 目录下的所有 .launch.py 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装 rviz 目录下的所有 .rviz 文件
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['front_wheels_node = car_motor_control.front_wheels_node:main',
                            'four_wheels_node = car_motor_control.four_wheels_node:main',
                            'diff_control_node = car_motor_control.diff_control_node:main',
                            'diff_cmdvel_node = car_motor_control.diff_cmdvel_node:main',
                            'diff_read_node = car_motor_control.diff_read_node:main',
                            'diff_param_node = car_motor_control.diff_param_node:main',
                            'diff_odom_node = car_motor_control.diff_odom_node:main',
        ],
    },
)
