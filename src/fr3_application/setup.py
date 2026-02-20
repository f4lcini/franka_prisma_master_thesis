import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fr3_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registrazione del pacchetto nell'indice delle risorse di ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Installazione del manifest del pacchetto
        ('share/' + package_name, ['package.xml']),
        
        # Esposizione rigorosa della directory di launch al sistema di build
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user', # Inserire il proprio nome
    maintainer_email='user@todo.todo', # Inserire la propria email
    description='Architettura applicativa per task di Pick and Place con Franka FR3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sintassi: nome_eseguibile = nome_pacchetto.nome_modulo:funzione_main
            # Questo definisce l'eseguibile 'fr3_pnp_node' invocato nel file run_pnp.launch.py
            'fr3_pnp_node = fr3_application.fr3_pnp_node:main'
        ],
    },
)