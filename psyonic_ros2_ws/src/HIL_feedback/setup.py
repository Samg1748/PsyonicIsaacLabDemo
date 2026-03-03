from setuptools import find_packages, setup

package_name = 'HIL_feedback'

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
    description='see package.xml',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'HIL_feedback = HIL_feedback.feedback:main'
        ],
    },
)
