from setuptools import find_packages, setup

package_name = 'mode_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mode_manager.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarohCodes',
    maintainer_email='aarohnanoti@gmail.com',
    description='DISABLED / MANUAL / AUTONOMOUS gate for /vehicle_command, with a deadman.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mode_manager_node = mode_manager.mode_manager_node:main',
        ],
    },
)
