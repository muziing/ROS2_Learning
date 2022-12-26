from setuptools import setup

package_name = 'py01_topic'

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
    maintainer_email='muzi2001@foxmial.com',
    description='话题通信实验案例',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo01_talker_str = py01_topic.demo01_talker_str:main',
            'demo02_listener_str = py01_topic.demo02_listener_str:main',
            'demo03_talker_stu = py01_topic.demo03_talker_stu:main',
            'demo04_listener_stu = py01_topic.demo04_listener_stu:main',
        ],
    },
)
