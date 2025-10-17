from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装配置文件到 share/controller/config/
        (os.path.join('share', package_name, 'config'), 
         glob('controller/config/*.yaml') + glob('controller/config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='za',
    maintainer_email='neu.ycl.za@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = controller.main:main',
        ],
    },
)
