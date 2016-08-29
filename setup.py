# PYTHON PACKAGING
# using setuptools : http://pythonhosted.org/setuptools/
import os
import sys
from setuptools import setup

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


if sys.argv[-1] == 'publish':

    os.system("python setup.py sdist")
    os.system("python setup.py bdist_wheel")
    # OLD way:
    #os.system("python setup.py sdist bdist_wheel upload")
    # NEW way:
    # Ref: https://packaging.python.org/distributing/
    os.system("twine upload dist/*")
    print("You probably want to also tag the version now:")
    print("  python setup.py tag")
    sys.exit()

if sys.argv[-1] == 'tag':
    os.system("git tag -a {0} -m 'version {0}'".format(__version__))
    os.system("git push --tags")
    sys.exit()

setup(name='pyros_setup',
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
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'six',
        'pyros_config>=0.1.2',
        'pytest>=2.5.1'
    ],
    setup_requires=[
        'pytest-runner'
    ],
    tests_require=[
    ],
    zip_safe=False,  # TODO testing...
)

