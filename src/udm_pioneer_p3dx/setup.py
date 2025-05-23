from setuptools import setup
from glob import glob
import os

package_name = 'udm_pioneer_p3dx'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'p3dx_description', 'meshes'), glob('p3dx_description/meshes/*')),
        (os.path.join('share', package_name, 'p3dx_gazebo'), glob('p3dx_gazebo/*')),
        (os.path.join('share', package_name, 'p3dx_navigation'), glob('p3dx_navigation/mult_nav.launch.py')),
        (os.path.join('share', package_name, 'p3dx_navigation'), glob('p3dx_navigation/*.yaml')),
        (os.path.join('share', package_name, 'p3dx_navigation'), glob('p3dx_navigation/*.xml')),
        (os.path.join('share', package_name, 'p3dx_navigation', 'config'), glob('p3dx_navigation/config/*')),
        (os.path.join('share', package_name, 'p3dx_navigation', 'maps'), glob('p3dx_navigation/maps/*'))
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
