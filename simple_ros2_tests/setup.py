from setuptools import setup

package_name = 'simple_ros2_tests'

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
    maintainer='belca',
    maintainer_email='belca@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'simple_publisher = simple_ros2_tests.simple_publisher:main',
        'minimal_test = simple_ros2_tests.minimal_test:main',
        'double_python = simple_ros2_tests.double_python:main',
        'ASP_publisher = simple_ros2_tests.ASP_publisher:main',
        'ASP_listener = simple_ros2_tests.ASP_listener:main',
    ],
},
)
