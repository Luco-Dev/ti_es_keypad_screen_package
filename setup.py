from setuptools import find_packages, setup

package_name = 'ti_es_keypad_screen_package'

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
    maintainer='luxo',
    maintainer_email='luco.berkouwer@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ti_es_keypad_screen_node = ti_es_keypad_screen_package.ti_es_keypad_screen_node:main'
        ],
    },
)
