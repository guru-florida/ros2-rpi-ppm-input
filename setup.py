from setuptools import setup
from glob import glob

package_name = 'rpi_ppm_input'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Colin F. MacKenzie',
    maintainer_email='colin@flyingeinstein.com',
    description='Publish a PPM signal such as a Radio Receiver on an RPi GPIO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ppm_input = rpi_ppm_input:main'
        ],
    },
)
