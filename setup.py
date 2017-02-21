# PYTHON PACKAGING
# using setuptools : http://pythonhosted.org/setuptools/
import os
import subprocess
import sys
import setuptools

with open('pyros_setup/_version.py') as vf:
    exec(vf.read())

# Best Flow :
# Clean previous build & dist
# $ gitchangelog >CHANGELOG.rst
# change version in code and changelog
# $ python setup.py prepare_release
# WAIT FOR TRAVIS CHECKS
# $ python setup.py publish
# => TODO : try to do a simpler "release" command


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class PrepareReleaseCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "prepare a release of pyros_setup"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""

        # TODO :
        # $ gitchangelog >CHANGELOG.rst
        # $ git commit CHANGELOG.rst -m "updating changelog"
        # change version in code and changelog
        subprocess.check_call("git commit CHANGELOG.rst pyros_setup/_version.py -m 'v{0}'".format(__version__), shell=True)
        subprocess.check_call("git push", shell=True)

        print("You should verify travis checks, and you can publish this release with :")
        print("  python setup.py publish")
        sys.exit()


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

        subprocess.check_call("python setup.py sdist", shell=True)
        subprocess.check_call("python setup.py bdist_wheel", shell=True)
        # OLD way:
        # os.system("python setup.py sdist bdist_wheel upload")
        # NEW way:
        # Ref: https://packaging.python.org/distributing/
        subprocess.check_call("twine upload dist/*", shell=True)

        subprocess.check_call("git tag -a {0} -m 'version {0}'".format(__version__), shell=True)
        subprocess.check_call("git push --tags", shell=True)
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
        'pyros_config>=0.2.0',
        'pytest>=2.5.1'
    ],
    setup_requires=[
        'pytest-runner',
        # 'twine'  #  this requires requests >= 2.5.0, which breaks on trusty...
    ],
    tests_require=[
    ],
    cmdclass={
        'prepare_release': PrepareReleaseCommand,
        'publish': PublishCommand,
    },
    zip_safe=True,
)

