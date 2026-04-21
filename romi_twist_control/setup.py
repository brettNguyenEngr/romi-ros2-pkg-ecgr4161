from setuptools import find_packages, setup

package_name = 'romi_twist_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brett',
    maintainer_email='brettnguyen.engr@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop = romi_twist_control.teleop_node:main',
            'i2c = romi_twist_control.i2c_node:main',
            'kinematics = romi_twist_control.kinematics_node:main',
            'pi = romi_twist_control.pi_node:main',
            'roomba = romi_twist_control.roomba_node:main',
            'odometry = romi_twist_control.odometry_node:main',
            'velocity_ctrl = romi_twist_control.velocity_ctrl_node:main'
        ],
    },
)
