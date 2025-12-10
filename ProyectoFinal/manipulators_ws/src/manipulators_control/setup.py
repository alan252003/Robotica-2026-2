from setuptools import find_packages, setup

package_name = 'manipulators_control'

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
    maintainer='robousr',
    maintainer_email='acpalacios.2503@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          "controller_manager_xy = manipulators_control.controller_manager_xy:main",
          "hardware_interface_xy = manipulators_control.hardware_interface_xy:main",
          "manipulator_controller_xy = manipulators_control.manipulator_controller_xy:main",
          "controller_manager_zy = manipulators_control.controller_manager_zy:main",
          "hardware_interface_zy = manipulators_control.hardware_interface_zy:main",
          "manipulator_controller_zy = manipulators_control.manipulator_controller_zy:main",
        ],
    },
)
