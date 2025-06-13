from setuptools import find_packages, setup

package_name = 'talos_mpc'

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
    maintainer='Pranav Debbad',
    maintainer_email='pranavdebbad31@gmail.com',
    description='Model predictive ROS2 controller using Crocoddyl for Talos humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = talos_mpc.mpc_node:main'
        ],
    },
)
