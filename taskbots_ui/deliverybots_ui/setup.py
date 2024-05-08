from setuptools import find_packages, setup

package_name = 'deliverybots_ui'

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
    maintainer='wale',
    maintainer_email='wale@iconbuild.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal = deliverybots_ui.goal_publisher:main',
            'command = deliverybots_ui.command_publisher:main',
            "setGoal_server = deliverybots_ui.set_integer_service_server:main",
            "setGoal_client = deliverybots_ui.set_integer_service_client:main",
        ],
    },
)
