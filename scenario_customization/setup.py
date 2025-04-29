from setuptools import find_packages, setup
import glob
import os

package_name = 'scenario_customization'


def get_task_directories(base_dir: str):
    """Function to dynamically get all task directories in the package"""
    task_dirs = []
    for root, dirs, files in os.walk(base_dir):
        # Look for directories that are tasks
        if "query_examples" in dirs or "explainability_examples" in dirs:
            task_dirs.append(os.path.basename(root))  # Add task name (task_1, task_2, etc.)
    return task_dirs

def build_data_files():
    """Function to dynamically build data files (all JSON files)"""
    task_dirs = get_task_directories('scenario_customization')
    data_files = []
    
    for task in task_dirs:
        query_examples = glob.glob(f'scenario_customization/{task}/query_examples/*.json', recursive=True)
        explainability_examples = glob.glob(f'scenario_customization/{task}/explainability_examples/*.json', recursive=True)

        if query_examples:
            data_files.append((os.path.join('share', package_name, 'examples', task, 'query_examples'), query_examples))
        if explainability_examples:
            data_files.append((os.path.join('share', package_name, 'examples', task, 'explainability_examples'), explainability_examples))
    
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *build_data_files(),
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
        ],
    },
)
