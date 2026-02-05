from setuptools import setup

package_name = 'ir2136'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pau',
    maintainer_email='al415491@uji.es',
    description='ROS2 nodes for battery + GPS control via pymavlink',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # OJO: usa el nombre REAL del .py sin .py
            'battery_gps_node = ir2136.battery_gps_node:main',
            'mission_control_node = ir2136.mission_control_node:main'
        ],
    },
)
