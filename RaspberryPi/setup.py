from distutils.core import setup
import py2exe, sys, os

sys.argv.append('py2exe')

setup(
    options = {'py2exe': {'optimize': 2}},
    windows = [{'script': "AR2.py","icon_resources": [(1, "AR2.ico")]}],
    zipfile = "shared.lib",
)