from setuptools import find_packages, setup

package_name = 'my_lidar_pkg'

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
    maintainer='chan',
    maintainer_email='cksgml8760@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_analysis = my_lidar_pkg.lidar_analysis:main',
            'lidar_sensor_analysis = my_lidar_pkg.lidar_sensor_analysis:main',
            'lidar_ml = my_lidar_pkg.lidar_ml:main'
        ],
    },
)
