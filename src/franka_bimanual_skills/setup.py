import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'franka_bimanual_skills'

setup(
    name='franka-bimanual-skills',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register the package in the ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Install the package manifest
        ('share/' + package_name, ['package.xml']),
        
        # Strict exposure of the launch directory to the build system
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user', # Insert your name here
    maintainer_email='user@todo.todo', # Insert your email here
    description='Application architecture for Pick and Place tasks with Franka FR3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Syntax: executable_name = package_name.module_name:main_function
            # This defines the executable 'fr3_pnp_node' invoked in run_pnp.launch.py
            'fr3_pnp_node = franka_bimanual_skills.fr3_pnp_node:main',
            'vlm_server_node = franka_bimanual_skills.vlm_server_node:main',
            'object_localization_node = franka_bimanual_skills.object_localization_node:main',
            'simple_moveit_server = franka_bimanual_skills.simple_moveit_server:main',
            'handover_coordinator = franka_bimanual_skills.handover_coordinator:main',
        ],
    },
)