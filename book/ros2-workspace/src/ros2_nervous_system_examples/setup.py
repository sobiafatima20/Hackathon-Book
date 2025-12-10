from setuptools import setup

package_name = 'ros2_nervous_system_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Book Team',
    maintainer_email='info@physicalai-book.org',
    description='ROS 2 examples for Physical AI & Humanoid Robotics Technical Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros2_nervous_system_examples.talker:main',
            'listener = ros2_nervous_system_examples.listener:main',
            'humanoid_controller = ros2_nervous_system_examples.humanoid_controller:main',
        ],
    },
)