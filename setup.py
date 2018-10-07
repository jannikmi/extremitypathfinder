# -*- coding:utf-8 -*-

import os
import re
import sys

from setuptools import setup


def get_version(package):
    """
    Return package version as listed in `__version__` in `__init__.py`.
    """
    init_py = open(os.path.join(package, '__init__.py')).read()
    return re.search("__version__ = ['\"]([^'\"]+)['\"]", init_py).group(1)


version = get_version('extremitypathfinder')
#
# with open('README.rst') as f:
#     readme = f.read()
#
# with open('CHANGELOG.rst') as changelog_file:
#     changelog = changelog_file.read()

# https://stackoverflow.com/questions/23174738/setup-py-packages-and-unicode-literals
native_string_pckg_name = 'extremitypathfinder'
if sys.version_info.major == 2:
    native_string_pckg_name = b'extremitypathfinder'

setup(
    name='extremitypathfinder',
    version=version,
    packages=['extremitypathfinder'],
    # package_data={
    #     native_string_pckg_name: [],
    # },
    description='python package for geometric shortest path computation for given 2D multi-polygon maps',
    author='J. Michelfeit',
    author_email='python@michelfe.it',
    license='MIT licence',
    url='https://github.com/MrMinimal64/extremitypathfinder',  # use the URL to the github repo
    keywords='path-planning path-finding shortest-path visibility graph polygon robotics navigation offline',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'Intended Audience :: Information Technology',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3 :: Only',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.0',
        'Programming Language :: Python :: 3.1',
        'Programming Language :: Python :: 3.2',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Topic :: Scientific/Engineering',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
    long_description='Python package for fast geometric shortest path computation in 2D multi-polygon '
                     'or grid environments based on visibility graphs.\n'
                     'Please check Github for the documentation with plots: '
                     'https://github.com/MrMinimal64/extremitypathfinder',
    # TODO
    # long_description=readme + '\n\n' + changelog,
    install_requires=[
        'numpy',
    ],
)
