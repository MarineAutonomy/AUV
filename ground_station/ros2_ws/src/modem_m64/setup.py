from setuptools import find_packages, setup

package_name = 'modem_m64'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aatmaj',
    maintainer_email='na22b018@smail.iitm.ac.in',
    description='Acoustic modem driver for Water Linked M64',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modem_node = modem_m64.modem_node:main',
        ],
    },
)
