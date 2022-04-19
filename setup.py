from setuptools import setup

setup(
    name='canfilter',
    version='0.0.0',
    description='canfilter module',
    author='Paul Ngo',
    author_email='ngopaul@berkeley.edu',
    packages=['canfilter'],
    install_requires=['matplotlib', 'pandas', 'numpy'],
    scripts=[
        'scripts/csv_replay.py'
    ]
)