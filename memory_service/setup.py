from setuptools import find_packages, setup

package_name = 'memory_service'

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
    maintainer='belca',
    maintainer_email='valeriobelcamino@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'memory_server = memory_service.memory_server:main',
            'memory_client = memory_service.memory_client:main',
            'memory_manage_llm = memory_service.memory_manage_llm:main',
        ],
    },
)
