from __future__ import absolute_import, division, print_function

# We need to be extra careful with python versions
# Ref : https://docs.python.org/dev/library/importlib.html#importlib.import_module

import os
import sys

# This will take the ROS distro version if ros has been setup
import genpy.generator
import genpy.generate_initpy


class ROSLoader(object):
    def __init__(self, generator, ext):
        self.ext = ext
        self.generator = generator

    # defining this to benefit from backward compat import machanism in python 3.X
    def get_filename(self, name):
        os.sep.join(name.split(".")) + '.' + self.ext

    # defining this to benefit from backward compat import machanism in python 3.X
    def is_package(self, name):
        return None  # TODO : implement check

    def load_module(self, fullname):


        return


class ROSFinder(object):

    def __init__(self):
        self.loaders = {
            '.srv': ROSLoader(genpy.generator.SrvGenerator(), 'srv'),
            '.msg': ROSLoader(genpy.generator.MsgGenerator(), 'msg')
        }

    def find_module(self, name, path=None):
        """
        Return the loader for the specified module.
        """
        # Ref : https://www.python.org/dev/peps/pep-0302/#specification-part-1-the-importer-protocol

        loader = None

        path = path or sys.path
        for p in path:
            for f in os.listdir(p):
                filename, ext = os.path.splitext(f)
                # our modules generated from messages are always a leaf in import tree so we only care about this case
                if ext in self.loaders.keys() and filename == name.split('.')[-1]:
                    loader = self.loaders.get(ext)
                    break  # we found it. break out.

        return loader


sys.meta_path += [ROSFinder()]
