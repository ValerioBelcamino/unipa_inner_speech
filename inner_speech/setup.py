from setuptools import setup

package_name = 'inner_speech'

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
    maintainer_email='valeriobelcamino@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'user_input = inner_speech.user_input:main',
        'llm_intent_recognition = inner_speech.llm_intent_recognition:main',        
        ],
    },
)
