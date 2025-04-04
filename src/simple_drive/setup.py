from os.path import join
from glob import glob
from setuptools import find_packages, setup

package_name = 'simple_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (join('share', 'ament_index', 'resource_index', 'packages'),
            [join('resource', package_name)]),
        (join('share', package_name), ['package.xml']),
        (join('share', package_name, 'launch'), glob(join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shailin-chavda',
    maintainer_email='shailinchavda@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sdm_navigator = simple_drive.sdm_navigator:main',
        ],
    },
)
