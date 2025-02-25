from setuptools import find_packages, setup

package_name = 'ur5e_control'

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
    maintainer='hri',
    maintainer_email='coronadozuniga.enrique@aist.go.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_control = ur5e_control.joint_control:main',
            'gripper_control = ur5e_control.gripper_control:main',
        ],
    },
)
