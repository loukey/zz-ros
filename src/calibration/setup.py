from setuptools import find_packages, setup

package_name = 'calibration'

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
    maintainer='za',
    maintainer_email='529768926@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera_calibration = calibration.camera_calibration:main',
            'undistort_node = calibration.undistort_node:main',
            'hand_eye_calibration = calibration.hand_eye_calibration:main',
            'robot_pose_simulator = calibration.robot_pose_simulator:main',
        ],
    },
)
