import os
from glob import glob
from setuptools import setup, find_packages

# Package directory structure
pkg_name = 'adisha_test'
pkg_list = [
    pkg_name,
    pkg_name + '/debug',
    pkg_name + '/example',
    pkg_name + '/research'
]

# Setup
setup(
    name                = pkg_name,
    version             = '0.0.0',
    packages            = pkg_list,
    data_files          = [
        ('share/ament_index/resource_index/packages', ['resource/' + pkg_name]),
        ('share/' + pkg_name, ['package.xml']),
        (os.path.join('share', pkg_name), glob('launch/*_launch.py'))
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'TODO: Add maintainer',
    maintainer_email    = 'TODO: Add maintainer email',
    description         = 'TODO: Package description',
    license             = 'TODO: License declaration',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': [
            'simple_publisher = adisha_test.example.simple_publisher:main',
            'simple_subscriber = adisha_test.example.simple_subscriber:main',
            'yaml_access = adisha_test.example.yaml_access:main'
        ],
    },
)
