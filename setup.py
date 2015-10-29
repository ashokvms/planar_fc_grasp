from __future__ import print_function
from setuptools import setup
from setuptools.command.test import test as TestCommand
import io
import os
import sys

here = os.path.abspath(os.path.dirname(__file__))

def read(*filenames, **kwargs):
    encoding = kwargs.get('encoding', 'utf-8')
    sep = kwargs.get('sep', '\n')
    buf = []
    for filename in filenames:
        with io.open(filename, encoding=encoding) as f:
            buf.append(f.read())
    return sep.join(buf)

long_description = read('README')

class PyTest(TestCommand):
    def finalize_options(self):
        TestCommand.finalize_options(self)
        self.test_args = []
        self.test_suite = True

    def run_tests(self):
        import pytest
        errcode = pytest.main(self.test_args)
        sys.exit(errcode)

setup(
    name='planar_fc_grasp',
    version='0.1',
    url='http://github.com/ashokvms/planar_fc_grasp/',
    license='Apache Software License',
    author='Ashok Meenakshi Sundaram',
    author_email='mashoksc@gmail.com.com',
    description='Foce Closure Grasp Analysis of Planar Objects SE(2)',
    long_description=long_description,
    packages=['planar_fc_grasp'],
    include_package_data=True,
    platforms='any',
    scripts=['scripts/planar_fc_grasp.py']
)