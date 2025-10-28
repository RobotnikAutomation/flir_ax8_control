
from setuptools import setup

setup(
    name='flir_ax8_control',
    version='0.1.0',
    packages=['flir'],
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'matplotlib',
    ],
    zip_safe=True,
)


