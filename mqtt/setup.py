from setuptools import find_packages, setup
from glob import glob

package_name = 'mqtt'

csv_files = glob('mqtt/mqtt/data/*.csv')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    package_data={'mqtt': ['data/*.csv']},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', csv_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='belca',
    maintainer_email='valeriobelcamino@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_publisher = mqtt.mqtt_json_publisher_test:main',
            'mqtt_subscriber = mqtt.mqtt_json_receiver:main',
        ],
    },
)
