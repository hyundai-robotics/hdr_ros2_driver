from setuptools import find_packages, setup
from glob import glob

package_name = 'hdr_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyojun',
    maintainer_email='kwon.hyojun@hd.com',
    description='Launch-only package',
    license='BSD-3-Clause',
)
