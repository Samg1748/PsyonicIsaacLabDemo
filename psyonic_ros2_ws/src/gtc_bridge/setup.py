from setuptools import find_packages, setup

package_name = 'gtc_bridge'

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
    maintainer='samg1748',
    maintainer_email='sgossett2020@gmail.com',
    description='see package.XML',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gtc_bridge = gtc_bridge.bridge:main'
        ],
    },
)
