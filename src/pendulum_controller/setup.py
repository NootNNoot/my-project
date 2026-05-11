from setuptools import setup
from glob import glob

package_name = 'pendulum_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            'share/' + package_name + '/config',
            glob('config/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Pendulum balancing controller',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'pendulum_balancer = pendulum_controller.pendulum_balancer:main',
            'auto_tuner = pendulum_controller.auto_tuner:main',
            'switcher = pendulum_controller.switcher:main',
            'torque_test = pendulum_controller.torque_test:main',
        ],
    },
)