from setuptools import setup

package_name = 'boat_location_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={package_name: package_name},
    data_files=[('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fake_boat_location = boat_location_publisher.fake_boat_location:main'
        ],
    },
)
