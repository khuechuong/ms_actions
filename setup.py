from setuptools import find_packages, setup

package_name = 'ms_actions'

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
    maintainer='erdc',
    maintainer_email='chuongl@unr.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_goal_server = ms_actions.go_to_goal_server:main',
            'multi_goal_client = ms_actions.multi_goal_client:main',
            'xarm_client_test = ms_actions.xarm_client_test:main',
            'inspect_client = ms_actions.inspect_client:main',
            'inspect_server = ms_actions.inspect_server:main',
        ],
    },
)
