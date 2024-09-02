#!/usr/bin/env python
import os
import sys
import subprocess
from parse import *

build_folder = sys.argv[1]
print('Called with build folder: "{}"'.format(build_folder))
os.chdir(build_folder)

make_cmd = 'make -j4'
if len(sys.argv) > 2 and sys.argv[2] == 'install':
    make_cmd += ' install'


def build_install_with_python_version(ver, make_cmd):

    # get python info
    output = subprocess.check_output('python{} --version'.format(ver),
                                     shell=True,
                                     stderr=subprocess.STDOUT)
    py_ver = parse('Python {}.{}.{}', output)
    py_ver_str = '{}.{}'.format(py_ver[0], py_ver[1])
    py_exec = subprocess.check_output('readlink -f $(which python{})'.format(ver),
                                       shell=True,
                                       stderr=subprocess.STDOUT)
    py_exec = py_exec.replace('\n', '')
    print('Python {} version is {}, executable at {}'.format(ver, py_ver_str, py_exec))


    # build python bindings
    cmd = 'cmake -DPYBIND11_PYTHON_VERSION={} -DPYTHON_EXECUTABLE={} .'.format(py_ver_str, py_exec)
    print('Running command "{}"'.format(cmd))
    if os.system(cmd) != 0:
        raise SystemError('Command "{}" failed'.format(cmd))

    cmd = make_cmd
    if os.system(cmd) != 0:
        raise SystemError('Command "{}" failed'.format(cmd))


build_install_with_python_version(2, make_cmd)
build_install_with_python_version(3, make_cmd)