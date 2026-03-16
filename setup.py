from setuptools import setup

package_name = 'ros_states'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros_states.launch.py']),
        ('share/' + package_name + '/templates', ['templates/index.html']),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Web-based real-time monitoring dashboard for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros_states_server = ros_states.app:main',
        ],
    },
)
