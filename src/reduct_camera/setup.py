from setuptools import find_packages, setup

package_name = 'reduct_camera'

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
    maintainer='anthony',
    maintainer_email='info@reduct.store',
    description='Simple setup for using ROS 2 with ReductStoren to store camera images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_and_store = reduct_camera.capture_and_store:main'
        ],
    },
)
