from setuptools import find_packages, setup
import os, glob

package_name = 'sign_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hri',
    maintainer_email='enriquecoronadozu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_proc = sign_agent.img_proc:main',
            'img_pub = sign_agent.img_pub:main',
            'machine_state = sign_agent.machine_state:main',
        ],
    },
)
