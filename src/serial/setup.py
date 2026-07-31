from setuptools import find_packages, setup

package_name = 'serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='galbrena',
    maintainer_email='wavesyang@outlook.com',
    description='HC-12 serial bridge for UAV-car-GCS communication',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge = serial.serial_bridge_node:main',
        ],
    },
)
