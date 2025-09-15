from setuptools import setup

package_name = 'quin_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[
        'cmd_move',
        'joystick_control',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='earn',
    maintainer_email='earnearn.panuditeekun@gmail.com',
    description='ROS2 package for controlling robot with PS4 joystick and ESP32',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_move = cmd_move.py:main',
            'joystick_control = joystick_control:main',
        ],
    },
)
