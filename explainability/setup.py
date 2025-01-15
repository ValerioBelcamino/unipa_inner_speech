from setuptools import setup

package_name = 'explainability'

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
        'explainability_listener = explainability.explainability_listener:main',
        'llm_query_explaination = explainability.llm_query_explaination:main',
        'llm_clingo_explaination = explainability.llm_clingo_explaination:main',        
        ],
    },
)
