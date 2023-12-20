from setuptools import find_packages, setup

package_name = 'robocup_dashboard'

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
    maintainer='hazel',
    maintainer_email='bjxobjxo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_first_node = robocup_dashboard.my_first_node:main",
            "draw_circle = robocup_dashboard.draw_circle:main",
            "pose_subscriber = robocup_dashboard.pose_subscriber:main",
            "turtle_controller = robocup_dashboard.turtle_controller:main",
            "dashboard = robocup_dashboard.main:main",
        ],
    },
)
