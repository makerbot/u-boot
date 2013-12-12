import os

env = Environment(ENV = os.environ)

ubootDir = os.path.abspath(str(Dir('#')))
baseDir = os.path.abspath(os.path.join(ubootDir, os.pardir))

subenv = os.environ.copy()

def path_add(env, key, path):
    if key in env:
        env[key] = path + os.pathsep + env[key]
    else:
        env[key] = path

# U-boot uses the angstrom toolchain
angstrom = os.path.join(baseDir, 'Birdwing-Cross-Compile-Tools', 'angstrom', 'arm')
tool_prefix = 'arm-angstrom-linux-gnueabi'
env.PrependENVPath('PATH', os.path.join(angstrom, 'bin'))
env.PrependENVPath('CPATH', os.path.join(angstrom, tool_prefix, 'usr', 'include'))
env['ENV']['LIBTOOL_SYSROOT_PATH'] = os.path.join(angstrom, tool_prefix)
env['ENV']['PKG_CONFIG_SYSROOT_DIR'] = os.path.join(angstrom, tool_prefix)
env['ENV']['PKG_CONFIG_PATH'] = os.path.join(angstrom, tool_prefix, 'usr', 'lib', 'pkgconfig')
env['ENV']['CONFIG_SITE'] = os.path.join(angstrom, 'site-config')

# Yes, this is a scons script to call make
def make_cmd(*args):
    makebaseCmd = ['make',
        'CROSS_COMPILE=%s-' % (tool_prefix),
    ]
    makeCmd = makebaseCmd + list(args)
    return ' '.join(makeCmd)

config_sources = [
    'include/configs/mb_manhattan.h'
]

config_targets = [
	'arch/arm/include/asm/arch',
	'arch/arm/include/asm/proc',
	'include/asm',
	'include/config.h',
	'include/config.mk',
]

env.Command(config_targets, config_sources, make_cmd('mb_manhattan_config'))

build_sources = config_targets

build_targets = [
    'u-boot.bin',
]

env.Command(build_targets, build_sources, make_cmd('u-boot.bin'))

#binary = os.path.join(ubootDir, 'u-boot.bin')
#out = os.path.join(ubootDir, 'u-boot-manhattan.bin')
#from add_uboot_header import UBootHeader
#p = UBootHeader(binary, out)
#p.add_header()
#p.close()
