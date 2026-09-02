from setuptools import find_packages, setup

package_name = 'quadrotor_teleop'

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
    maintainer='kunal',
    maintainer_email='kunalnk@udel.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quadrotor_teleop_node = quadrotor_teleop.quadrotor_teleop_node:main'
        ],
    },
)
