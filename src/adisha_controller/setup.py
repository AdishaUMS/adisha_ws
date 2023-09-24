import os
from glob import glob
from setuptools import setup

pkg_name = 'adisha_controller'
pkg_list = [
    pkg_name,
    pkg_name + '/submodule'
]

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
            'adisha_dxlsyncrwpos = adisha_controller.adisha_dxlsyncrwpos:main'
        ],
    },
)
