# -*- coding: utf-8 -*-
from distutils.core import setup
import py2exe

setup(options = {"py2exe":{   "excludes": ["_pytest","qtpy"] } },
      console = ['python_interface.py'])