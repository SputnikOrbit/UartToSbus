from setuptools import find_packages, setup

package_name = 'chassis_controller_channels'

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
    maintainer='sputnik',
    maintainer_email='22373482@buaa.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'velocity_publisher = chassis_controller_channels.ui_publisher:main',
        ],
    },
)
