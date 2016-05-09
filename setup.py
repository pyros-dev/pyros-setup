# PYTHON PACKAGING
# using setuptools : http://pythonhosted.org/setuptools/
from setuptools import setup

with open('pyros_setup/_version.py') as vf:
    exec(vf.read())

setup(name='pyros_setup',
    version='0.1.0',
    description='Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python',
    url='http://github.com/asmodehn/pyros-setup',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='BSD',
    packages=[
        'pyros_setup',
        'pyros_setup.config',
        'pyros_setup.tests',
    ],
    entry_points={
        'console_scripts': [
            'pyros_setup = pyros_setup.__main__:nosemain'
        ]
    },
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        #'catkin_pkg',  # not needed here since this version should not look for package.xml
        'six',
        'importlib'
    ],
    test_suite="nose.collector",
    tests_require=[
        'nose>=1.3.7'
    ],
    zip_safe=False,  # TODO testing...
)

