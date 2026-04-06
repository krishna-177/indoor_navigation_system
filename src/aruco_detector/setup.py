from setuptools import setup

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='krishna@example.com',
    description='ArUco detection from IP camera stream',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_detector.aruco_node:main',
            'aruco_xy_localization = aruco_detector.aruco_xy_localization:main',
            'click_to_world = aruco_detector.click_to_world_node:main',
            'reference_click_localization = aruco_detector.reference_click_localization:main',
            'aruco_homography_localization = aruco_detector.aruco_homography_localization:main',
            'aruco_robot_pose = aruco_detector.aruco_robot_pose:main',
            'goal_click_node = aruco_detector.goal_click_node:main',
            'controller_node = aruco_detector.controller_node:main',
        ],
    },
)
