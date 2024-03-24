from setuptools import setup

package_name = 'lidar_pubsub'

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
    maintainer='evanyan13',
    maintainer_email='evanyan13@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    	'console_scripts': [
                'talker = lidar_pubsub.publisher_lidar_function:main',
                'listener = lidar_pubsub.subscriber_lidar_function:main',
        ],
    },
)
