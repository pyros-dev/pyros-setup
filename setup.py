# PYTHON PACKAGING
# using setuptools : http://pythonhosted.org/setuptools/
import os
import sys
import setuptools

with open('pyros_setup/_version.py') as vf:
    exec(vf.read())

# Best Flow :
# git changelog >CHANGELOG.rst
# git commit "updating changelog"
# change version in code and changelog
# git commit "v<M.m.p>"
# python setup.py publish
# python setup.py tag
# => try to do a simple "release" command


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class PublishCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "releases pyros_setup to Pypi"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""

        os.system("python setup.py sdist")
        os.system("python setup.py bdist_wheel")
        # OLD way:
        # os.system("python setup.py sdist bdist_wheel upload")
        # NEW way:
        # Ref: https://packaging.python.org/distributing/
        os.system("twine upload dist/*")
        print("You probably want to also tag the version now:")
        print("  python setup.py tag")
        sys.exit()


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class TagCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "tag a release of pyros_setup"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""

        os.system("git tag -a {0} -m 'version {0}'".format(__version__))
        os.system("git push --tags")
        sys.exit()


setuptools.setup(name='pyros_setup',
    version=__version__,
    description='Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python',
    url='http://github.com/asmodehn/pyros-setup',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='BSD',
    packages=[
        'pyros_setup',
        'pyros_setup.tests',
    ],
    entry_points={
        'console_scripts': [
            'pyros_setup = pyros_setup.__main__:main'
        ]
    },
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in when using source dist.
    install_requires=[
        'six',
        'pyros_config>=0.1.4',
        'pytest>=2.5.1'
    ],
    setup_requires=[
        'pytest-runner'
    ],
    tests_require=[
    ],
    cmdclass={
         'tag': TagCommand,
         'publish': PublishCommand,
    },
    zip_safe=True,
)

