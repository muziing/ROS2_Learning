from setuptools import setup

package_name = 'py04_param'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muzing',
    maintainer_email='muzi2001@foxmail.com',
    description='服务参数实验',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo00_param = py04_param.demo00_param:main',
            'demo01_param_server = py04_param.demo01_param_server:main',
        ],
    },
)
