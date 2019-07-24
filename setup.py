from setuptools import setup, find_packages

package_name = 'razor_imu_9dof'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[],
    install_requires=['setuptools', 'pyserial'],
    author='Abhishek N. Kulkarni',
    author_email='ankulkarni@wpi.edu',
    maintainer='Abhishek N. Kulkarni',
    maintainer_email='ankulkarni@wpi.edu',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 drivers for Sparkfun Razor IMU 9DOF. ',
    license='Apache License, Version 2.0',
    test_suite='pytest',
    entry_points={
        'console_scripts': [
            'imu_node = nodes.imu_node:main',
        ],
    },
)