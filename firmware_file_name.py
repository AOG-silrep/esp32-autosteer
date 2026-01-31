#!/usr/bin/python3

Import("env")

env.Replace(PROGNAME="autosteer_77_firmware_%s" % env.GetProjectOption("custom_prog_version"))

# Expose custom_prog_version to the compiler as a macro so source code can
# include the version in the UI.

v = env.GetProjectOption("custom_prog_version")
env.Append(CPPDEFINES=[('CUSTOM_PROG_VERSION', '\\\"%s\\\"' % v)])