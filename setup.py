from setuptools import setup

setup(
    name='TC1_motor_model',
    packages=[
        'TC1_motor_model',
    ],
    install_requires=[
        'numpy',
        'csdl',
    ],
    version_config=True,
)