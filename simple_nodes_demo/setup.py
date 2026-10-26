from setuptools import setup

package_name = 'simple_nodes_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kunal Chandmare',
    maintainer_email='kunal@example.com',
    description='Simple multi-node ROS 2 learning example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_publisher = simple_nodes_demo.battery_publisher:main',
            'battery_processor = simple_nodes_demo.battery_processor:main',
        ],
    },
)
