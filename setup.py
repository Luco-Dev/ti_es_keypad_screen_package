from setuptools import setup
import os

package_name = 'ti_es_keypad_screen_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='luxo',
    maintainer_email='luxo@example.com',
    description='ROS2 package for screen keypad module',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ti_es_keypad_screen_node = ti_es_keypad_screen_package.ti_es_keypad_screen_node:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    include_package_data=True,  # Ensure data files are included
)
