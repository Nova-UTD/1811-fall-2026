from setuptools import find_packages, setup

package_name = 'routing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/route_recorder.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adit22',
    maintainer_email='k28adit@gmail.com',
    description='Teach-and-repeat route recording for the 1811 vehicle.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'route_recorder_node = routing.route_recorder_node:main',
        ],
    },
)
