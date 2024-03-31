from setuptools import setup, find_packages

package_name = 'navi_main'

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
    maintainer='evanyan13',
    maintainer_email='weidongy201@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner_main = navi_main.global_planner_main:main',
            'global_planner_user = navi_main.global_planner_user:main',
            'debug_node = navi_main.debug_node:main',
        ],
    },
)
