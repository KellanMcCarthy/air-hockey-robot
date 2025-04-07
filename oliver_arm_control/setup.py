from setuptools import find_packages, setup

package_name = 'oliver_arm_control'

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
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='Apahe-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_control = oliver_arm_control.arm_control:main'
        ],
    },
)
