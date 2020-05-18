# -*- coding:utf-8 -*-

from setuptools import setup

setup(
    name='extremitypathfinder',
    packages=['extremitypathfinder'],
    description='python package for geometric shortest path computation for given 2D multi-polygon maps',
    # version: in VERSION file https://packaging.python.org/guides/single-sourcing-package-version/
    # With this approach you must make sure that the VERSION file is included in all your source
    # and binary distributions (e.g. add include VERSION to your MANIFEST.in).
    author='J. Michelfeit',
    author_email='python@michelfe.it',
    license='MIT licence',
    url='https://github.com/MrMinimal64/extremitypathfinder',  # use the URL to the github repo
    project_urls={
        "Source Code": "https://github.com/MrMinimal64/extremitypathfinder",
        "Documentation": "https://github.com/MrMinimal64/extremitypathfinder/blob/master/README.rst",
        "Changelog": "https://github.com/MrMinimal64/extremitypathfinder/blob/master/CHANGELOG.rst",
    },
    keywords='path-planning path-finding shortest-path visibility graph visibility-graph polygon'
             'robotics navigation offline ',
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
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Topic :: Scientific/Engineering',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
    long_description='python package for fast geometric shortest path computation in 2D multi-polygon '
                     'or grid environments based on visibility graphs.'
                     'Please refer to the `documentation <https://extremitypathfinder.readthedocs.io/en/latest/>`__.',
    python_requires='>=3.7',
    install_requires=[
        'numpy>=1.16',
    ],
)
