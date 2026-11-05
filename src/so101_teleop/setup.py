from setuptools import find_packages, setup
from glob import glob

package_name = 'so101_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models/so101',
            glob('models/so101/*.xml') + ['models/so101/LICENSE']),
        ('share/' + package_name + '/models/so101/assets',
            glob('models/so101/assets/*.stl')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Azka Adziman',
    maintainer_email='azka.adziman@exeter.ox.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'leader_node = so101_teleop.leader_node:main',
        'print_ticks = so101_teleop.leader_node:print_ticks',
        'panda_sim_node = so101_teleop.panda_sim_node:main',
        'retarget_node = so101_teleop.retarget_node:main',
        'panda_ik_sim_node = so101_teleop.panda_ik_sim_node:main',
        'latency = so101_teleop.latency:main',
        'so101_sim_node = so101_teleop.so101_sim_node:main',
        'cube_rush_node = so101_teleop.cube_rush_node:main',
        'follower_node = so101_teleop.follower_node:main',
        ],
    },
)
