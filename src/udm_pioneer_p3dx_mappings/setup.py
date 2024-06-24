from setuptools import setup
from glob import glob
import os

package_name = 'udm_pioneer_p3dx_mappings'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'pioneer_p3dx_description', 'meshes'), glob(os.path.join('pioneer_p3dx_description', 'meshes', '*'))),
        (os.path.join('share', package_name, 'pioneer_p3dx_description', 'urdf'), glob(os.path.join('pioneer_p3dx_description', 'urdf', '*'))),
        (os.path.join('share', package_name, 'pioneer_p3dx_description', 'config'), glob(os.path.join('pioneer_p3dx_description', 'config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ara',
    maintainer_email='oladipoeyiara@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
