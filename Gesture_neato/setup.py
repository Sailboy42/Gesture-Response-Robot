from setuptools import find_packages, setup

package_name = 'Gesture_neato'

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
    maintainer='tabby305',
    maintainer_email='tdavison@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'gesture_fsm = Gesture_neato.Gesture_neato.Gesture_FSM:main',
        'gest_recog_camera = Gesture_neato.Gesture_neato.gest_recog_camera:main',
    ],
},
)
