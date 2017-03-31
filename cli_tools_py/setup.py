from setuptools import setup

setup(
    name='cli_tools_py',
    version='0.0.0',
    packages=[],
    py_modules=[
        'rosnode_list',
        'rostopic_echo',
        'rostopic_list',
        'rostopic_pub',
    ],
    install_requires=['setuptools'],
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Set of rostopic tools using rclpy API.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'rosnode_list_py = rosnode_list:main',
            'rostopic_echo_py = rostopic_echo:main',
            'rostopic_list_py = rostopic_list:main',
            'rostopic_pub_py = rostopic_pub:main',
        ],
    },
)
