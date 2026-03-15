from setuptools import find_packages, setup

package_name = 'pub_sub_example'

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
    description='Intro string publisher/subscriber package for ECGR 4161 Lab 1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pub_sub_example.str_publisher:main',
            'listener = pub_sub_example.str_subscriber:main',
        ],
    },
)
