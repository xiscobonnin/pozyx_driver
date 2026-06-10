from setuptools import find_packages, setup

package_name = 'pozyx_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pozyx_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xisco',
    maintainer_email='xisco.bonnin@uib.es',
    description='The pozyx_driver package',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pozyx_driver_node = pozyx_driver.pozyx_driver_node:main',
        ],
    },
)
