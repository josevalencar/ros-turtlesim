from setuptools import find_packages, setup

package_name = 'ponderada_turtlesim'

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
    maintainer='josevalencar',
    maintainer_email='jose.silva@sou.inteli.edu.br',
    description='Package para ponderada de turtlesim do inteli',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "zeturguita = ponderada_turtlesim.ros_zeturguita:main",
        ],
    },
)
