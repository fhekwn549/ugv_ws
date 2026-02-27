from setuptools import find_packages, setup

package_name = 'ugv_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['schema.sql']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/bridge.launch.py']),
        ('share/' + package_name + '/config',
            ['config/bridge_params.yaml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='fhekwn549',
    maintainer_email='fhekwn549@gmail.com',
    description='MQTT + REST API bridge between ROS 2 and web UI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = ugv_bridge.bridge_node:main',
        ],
    },
)
