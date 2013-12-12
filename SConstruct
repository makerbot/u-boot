import os
import fnmatch

env = Environment(ENV = os.environ)

#TODO: Use mw_scons_tools rather than copy/pasting this
# This is a special glob made by NicholasBishop
def mb_recursive_file_glob(env, root, pattern, exclude = None):
    """Recursively search in 'root' for files matching 'pattern'

    Returns a list of matches of type SCons.Node.FS.File.

    If exclude is not None, it should be a glob pattern or list of
    glob patterns. Any results matching a glob in exclude will be
    excluded from the returned list."""
    def excluded(filename, exclude):
        if exclude:
            if isinstance(exclude, str):
                exclude = [exclude]
            for pattern in exclude:
                if fnmatch.fnmatch(filename, pattern):
                    return True
        return False

    matches = []
    if root.startswith('#'):
        raise Exception('Directories starting with "#" not supported yet')
    project_root = env.Dir('#').abspath
    for parent, dirnames, filenames in os.walk(os.path.join(
            project_root, root)):
        for filename in fnmatch.filter(filenames, pattern):
            if not excluded(filename, exclude):
                p = os.path.join(parent, filename)
                rp = os.path.relpath(p, project_root)
                matches.append(env.File(rp))
    return matches

env.AddMethod(mb_recursive_file_glob, 'MBRecursiveFileGlob')

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
build_sources.extend(env.MBRecursiveFileGlob('.', '*.c'))
build_sources.extend(env.MBRecursiveFileGlob('.', '*.h'))
def not_generated(path):
    return not str(path).startswith('include/generated/')
build_sources = filter(not_generated, build_sources)

build_targets = [
    'u-boot.bin',
	'u-boot',
	'u-boot.lds',
	'u-boot.map',
]

build = env.Command(build_targets, build_sources, make_cmd('u-boot.bin'))

clean_targets = [
	'arch/arm/cpu/arm926ejs/davinci/asm-offsets.s',
	'include/autoconf.mk',
	'include/autoconf.mk.dep',
	'lib/asm-offsets.s',
	'tools/envcrc',
	'tools/gen_eth_addr',
	'tools/img2srec',
	'tools/kernel-doc/docproc',
	'tools/mkenvimage',
	'tools/mkimage',
	'tools/proftool',
]
clean_targets.extend(env.MBRecursiveFileGlob('include/generated', '*'))
clean_targets.extend(env.MBRecursiveFileGlob('.', '*.o'))
clean_targets.extend(env.MBRecursiveFileGlob('.', '.depend*'))

env.Clean(build, clean_targets)

#binary = os.path.join(ubootDir, 'u-boot.bin')
#out = os.path.join(ubootDir, 'u-boot-manhattan.bin')
#from add_uboot_header import UBootHeader
#p = UBootHeader(binary, out)
#p.add_header()
#p.close()
