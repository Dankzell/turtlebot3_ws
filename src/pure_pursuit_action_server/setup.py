from setuptools import find_packages, setup

package_name = 'pure_pursuit_action_server'

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
    maintainer='rosuser',
    maintainer_email='gustav.forsell@live.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_action_server_exe = pure_pursuit_action_server.pure_pursuit_action_server:main',
            'pure_pursuit_action_client_exe = pure_pursuit_action_server.pure_pursuit_action_client:main'
        ],
    },
)
