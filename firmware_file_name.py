#!/usr/bin/python3

Import("env")

env.Replace(PROGNAME="autosteer_77_firmware_%s" % env.GetProjectOption("custom_prog_version"))