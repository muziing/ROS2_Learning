from setuptools import setup

package_name = 'py02_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muzing',
    maintainer_email='muzi2001@foxmail.com',
    description='服务通信实验',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo01_server = py02_service.demo01_server:main',
            'demo02_client = py02_service.demo02_client:main',
        ],
    },
)
