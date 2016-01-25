# This setup is usable by catkin, or on its own as usual python setup.py

_CATKIN = False
try:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup
    _CATKIN = True
except Exception, e:
    from setuptools import setup

# CAREFUL distutils and setuptools take different arguments and have different behaviors
# ROS PACKAGING
if _CATKIN:  # using distutils : https://docs.python.org/2/distutils
    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=[
            'pyros_setup',
            'pyros_setup.tests',  # including tests to be able to validate behavior of installed package
        ],
    )
    setup(**setup_args)

# PYTHON PACKAGING
else:  # using setuptools : http://pythonhosted.org/setuptools/

    setup(name='pyros_setup',
        version='0.0.9',
        description='Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python',
        url='http://github.com/asmodehn/pyros-setup',
        author='AlexV',
        author_email='asmodehn@gmail.com',
        license='BSD',
        packages=[
            'pyros_setup',
            'pyros_setup.tests',
        ],
        # this is better than using package data ( since behavior is a bit different from distutils... )
        include_package_data=True,  # use MANIFEST.in during install.
        install_requires=[
            #'catkin_pkg',  # not needed here since this version should not look for package.xml
        ],
        test_requires=[
            'pyros_test'
        ],
        zip_safe=False,  # TODO testing...
    )

