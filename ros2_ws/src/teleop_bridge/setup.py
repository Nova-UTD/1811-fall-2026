from setuptools import find_packages, setup

package_name = 'teleop_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial', 'pygame'],
    zip_safe=True,
    maintainer='adit22',
    maintainer_email='k28adit@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gamepad_node = teleop_bridge.gamepad_node:main',
            'serial_bridge_node = teleop_bridge.serial_bridge_node:main',
            'keyboard_teleop_node = teleop_bridge.keyboard_teleop_node:main',
        ],
    },
)
