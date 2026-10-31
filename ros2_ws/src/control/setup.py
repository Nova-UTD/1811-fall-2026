from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pure_pursuit.launch.py']),
        ('share/' + package_name + '/config', [
            'config/pure_pursuit.yaml',
            'config/sample_loop.csv',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adit22',
    maintainer_email='k28adit@gmail.com',
    description='Pure pursuit path-tracking controller for the 1811 vehicle.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = control.pure_pursuit_node:main',
            'bicycle_sim_node = control.bicycle_sim_node:main',
        ],
    },
)
