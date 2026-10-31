from setuptools import find_packages, setup

package_name = 'camera_panorama'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/panorama.yaml']),
        ('share/' + package_name + '/launch', ['launch/panorama.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarohCodes',
    maintainer_email='aarohnanoti@gmail.com',
    description='360-degree panorama from the four 1811 ZED X cameras.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'panorama_node = camera_panorama.panorama_node:main',
        ],
    },
)
