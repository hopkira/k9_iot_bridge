from setuptools import setup

package_name = 'k9_iot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='MQTT bridge between Bangle.js joystick and /cmd_vel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # this must match your package + module + main()
            'k9_iot_bridge_node = k9_iot_bridge.k9_iot_bridge_node:main',
        ],
    },
)
