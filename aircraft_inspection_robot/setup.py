from setuptools import setup, find_packages

package_name = 'aircraft_inspection_robot'

setup(
    name=package_name,
    version='0.0.0',
    # Needed to do relative imports of other python files in the package
    # Probably a better way to do it 
    packages=find_packages(exclude="test"),  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='rpo1202@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
)