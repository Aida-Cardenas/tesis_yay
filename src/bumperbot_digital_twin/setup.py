from glob import glob

from setuptools import setup

package_name = 'bumperbot_digital_twin'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aida Cárdenas',
    maintainer_email='aida.cardenas@correo.unimet.edu.ve',
    description='Capa de comunicación bidireccional entre el robot físico y su gemelo digital',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twin_bridge = bumperbot_digital_twin.twin_bridge:main',
            'fake_robot = bumperbot_digital_twin.fake_robot:main',
            'scripted_driver = bumperbot_digital_twin.scripted_driver:main',
            'latency_echo = bumperbot_digital_twin.latency_echo:main',
            'analyze_twin_log = bumperbot_digital_twin.analyze_log:main',
            'anomaly_detector = bumperbot_digital_twin.anomaly_detector:main',
            'twin_calibrate = bumperbot_digital_twin.calibrate:main',
            'twin_report = bumperbot_digital_twin.report:main',
            'twin_experiment = bumperbot_digital_twin.experiment_runner:main',
            'twin_dashboard = bumperbot_digital_twin.dashboard:main',
            'twin_tune = bumperbot_digital_twin.tune:main',
            'twin_navigate = bumperbot_digital_twin.nav_preview:main',
            'fake_navigator = bumperbot_digital_twin.fake_navigator:main',
            'twin_odom_meter = bumperbot_digital_twin.odom_meter:main',
        ],
    },
)
