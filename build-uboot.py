#!/usr/bin/env python

# syntax: ./build_uboot.py [clean]

import sys
import os
import subprocess
import shutil

ubootDir = os.path.dirname(os.path.abspath(__file__))
baseDir = os.path.abspath(os.path.join(ubootDir, os.pardir, os.pardir))

subenv = os.environ.copy()

def path_add(env, key, path):
    if key in env:
        env[key] = path + os.pathsep + env[key]
    else:
        env[key] = path

# Linux uses the angstrom toolchain
angstrom = os.path.join(baseDir, 'toolchain', 'angstrom', 'arm')
tool_prefix = 'arm-angstrom-linux-gnueabi'
path_add(subenv, 'PATH', os.path.join(angstrom, 'bin'))
path_add(subenv, 'CPATH', os.path.join(angstrom, tool_prefix, 'usr', 'include'))
subenv['LIBTOOL_SYSROOT_PATH'] = os.path.join(angstrom, tool_prefix)
subenv['PKG_CONFIG_SYSROOT_DIR'] = os.path.join(angstrom, tool_prefix)
subenv['PKG_CONFIG_PATH'] = os.path.join(angstrom, tool_prefix, 'usr', 'lib', 'pkgconfig')
subenv['CONFIG_SITE'] = os.path.join(angstrom, 'site-config')

def do_make_cmd(args, path = ubootDir):
    makebaseCmd = ['make',
        'CROSS_COMPILE=%s-' % (tool_prefix),
    ]
    makeCmd = makebaseCmd + args
    print(' '.join(makeCmd))
    subprocess.check_call(makeCmd, env = subenv, cwd = path)

if 'clean' in sys.argv:
    print('Cleaning uboot...')
    do_make_cmd(['clean'])
    sys.exit(0)

print('Building u-boot...')
do_make_cmd(['mb_manhattan_config'])
do_make_cmd(['all'])

binary = os.path.join(ubootDir, 'u-boot.bin')
out = os.path.join(ubootDir, 'u-boot-manhattan.bin')
from add_uboot_header import UBootHeader
p = UBootHeader(binary, out)
p.add_header()
p.close()
