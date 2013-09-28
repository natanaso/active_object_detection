"""libANN (Apriximate Nearest Neighbors library) python interface
Copyright (C) 2008, Robert Hetland
"""

classifiers = """\
Development Status :: beta
Environment :: Console
Intended Audience :: Science/Research
Intended Audience :: Developers
License :: MIT
Operating System :: OS Independent
Programming Language :: Python
Topic :: Scientific/Engineering
Topic :: Software Development :: Libraries :: Python Modules
"""

PKG = 'ann2' # ROS package name

import roslib; roslib.load_manifest(PKG)
import os
from numpy.distutils.core import Extension

def GetANNDirectroyFromROS():
    annpkg = 'ann2'
    manifest_file = roslib.manifest.manifest_file(annpkg, True, os.environ)
    if not manifest_file:
        raise roslib.packages.InvalidROSPkgException("cannot locate package [%s]"%pkg)    
    return os.path.dirname(os.path.abspath(manifest_file))

libann_wrap = Extension(name = '_libann_wrap',
                sources = ['ann/ann_wrap.cpp'],
                include_dirs=[os.path.join(GetANNDirectroyFromROS(), 'include')],
                library_dirs=[os.path.join(GetANNDirectroyFromROS(), 'lib')],
                libraries=['ANN_1_1_1'])

doclines = __doc__.split("\n")

if __name__ == '__main__':
    from numpy.distutils.core import setup
    setup(name = "ann",
          version = '1.0',
          description = doclines[0],
          long_description = "\n".join(doclines[2:]),
          author = "Robert Hetland",
          author_email = "hetland@tamu.edu",
          packages = ['ann',],
          license = 'MIT',
          platforms = ["any",],
          ext_modules = [libann_wrap,],
          classifiers = filter(None, classifiers.split("\n")),
          )
    
