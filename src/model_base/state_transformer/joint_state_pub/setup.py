from setuptools import find_packages, setup

package_name = 'joint_state_pub'

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
    maintainer_email='za@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_state_pub = joint_state_pub.joint_state_pub:main',
        ],
    },
)
