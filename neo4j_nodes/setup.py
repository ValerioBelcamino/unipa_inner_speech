from setuptools import setup

package_name = 'neo4j_nodes'

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
        'export_query_results = neo4j_nodes.export_query_results:main',
        'llm_neo4j_query = neo4j_nodes.llm_neo4j_query:main',        
        'populate_database = neo4j_nodes.populate_database:main',        
        'clingo_publisher = neo4j_nodes.clingo_publisher:main',
        ],
    },
)
