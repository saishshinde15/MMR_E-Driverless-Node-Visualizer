from setuptools import find_packages, setup

package_name = 'mmr_ros_graph'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'static/index.html',
            'static/style.css',
            'static/app.js',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    zip_safe=False,
    maintainer='saish',
    maintainer_email='392180@studenti.unimore.it',
    description='Live browser view of a ROS 2 graph.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'live_graph = mmr_ros_graph.app:main',
        ],
    },
)
