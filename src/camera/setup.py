from setuptools import find_packages, setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='za',
    maintainer_email='529768926@qq.com',
    description='Camera calibration package for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibration = camera.camera_calibration:main',
            'undistort_node = camera.undistort_node:main',
            'hand_eye_calibration = camera.hand_eye_calibration:main',
        ],
    },
)
