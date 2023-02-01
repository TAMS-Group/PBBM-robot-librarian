#!/usr/bin/env python3

import setuptools
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['librarian_craft', 'librarian_craft.basenet', 'librarian_dtrb',
              'librarian_dtrb.modules', 'librarian_segmentation', 'librarian_monitor'],
    # include_package_data=True,
    package_dir={'': 'src'}
    # scripts=['models/detection_node.py']
)

setuptools.setup(**d)
