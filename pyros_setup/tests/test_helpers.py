# -*- coding: utf-8 -*-
"""
    test_helpers
    ~~~~~~~~~~~~
"""

# From flask tst.helpers. maybe useful for us as well ?
#
# import flask
#
# class TestNoImports(object):
#     """Test Flasks are created without import.
#
#     Avoiding ``__import__`` helps create Flask instances where there are errors
#     at import time.  Those runtime errors will be apparent to the user soon
#     enough, but tools which build Flask instances meta-programmatically benefit
#     from a Flask which does not ``__import__``.  Instead of importing to
#     retrieve file paths or metadata on a module or package, use the pkgutil and
#     imp modules in the Python standard library.
#     """
#
#     def test_name_with_import_error(self, modules_tmpdir):
#         modules_tmpdir.join('importerror.py').write('raise NotImplementedError()')
#         try:
#             flask.Flask('importerror')
#         except NotImplementedError:
#             assert False, 'Flask(import_name) is importing import_name.'

