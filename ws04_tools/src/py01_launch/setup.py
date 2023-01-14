from setuptools import setup
from glob import glob

package_name = 'py01_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name, ['launch/py/py01_helloworld_launch.py']),
        # ('share/' + package_name, ['launch/xml/xml01_helloworld_launch.xml']),
        # ('share/' + package_name, ['launch/yaml/yaml01_helloworld_launch.yaml']),
        ('share/' + package_name, glob("launch/py/*_launch.py")),
        ('share/' + package_name, glob("launch/xml/*_launch.xml")),
        ('share/' + package_name, glob("launch/yaml/*_launch.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muzing',
    maintainer_email='muzi2001@foxmail.com',
    description='launch文件学习',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [],
    },
)
