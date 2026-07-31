from setuptools import find_packages, setup

package_name = 'crosshair_aligner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Crosshair alignment using intensity-weighted centroid',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'align_node = crosshair_aligner.align_node:main',
            'cv2_camera_node = crosshair_aligner.cv2_camera_node:main',
        ],
    },
)
