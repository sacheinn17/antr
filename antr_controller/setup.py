from setuptools import find_packages, setup

package_name = 'antr_controller'

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
    maintainer='sac',
    maintainer_email='sac@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "runBot = antr_controller.simpleControl:main",
            "robot_localize = antr_controller.robot_localize:main",
            "antr_teleop_twist_keyboard = antr_controller.antr_teleop_twist_keyboard:main",
            "video_feed = antr_controller.video_feed:main",
            "control_robot = antr_controller.control_robot:main",
        ],
    },
)
