from setuptools import find_packages, setup

package_name = 'beamng_sensor_publisher'

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
    maintainer='arl',
    maintainer_email='willward1912@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'sensor_publisher = beamng_sensor_publisher.sensor_publisher:main',
        'sensor_publisher2 = beamng_sensor_publisher.sensor_publisher2:main',
    ],
},
)
