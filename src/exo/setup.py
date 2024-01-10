from setuptools import setup
import os
from glob import glob

package_name = 'exo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krita',
    maintainer_email='krita@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['control = exo.datasolver:main', 
                            'wlanesp = exo.wlanconn:main',
                            'interface = exo.interpreter:main'
        ],
    },
)
