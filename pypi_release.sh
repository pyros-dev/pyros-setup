#!/bin/sh

#TODO : find ~/.pypirc and extract repository
if [ $# -lt 1 ]; then
    echo "Usage : $0 <pypi|testpypi>"
    exit 127
fi

#extract version from setup.py
VERSION=`python setup.py --version`

#update package data
python setup.py register -r $1

#make sure version doesnt exist already
VERSION_EXISTS=`git tag | grep $VERSION | wc -l`
WORKING_TREE_CLEAN=`git status -s | wc -l`

if [ $VERSION_EXISTS -ne 1 ]; then
    echo git tag $VERSION doesnt not exist.
    if [ $WORKING_TREE_CLEAN -eq 0 ]; then
        echo Your working tree is not clean. Clean it up first !
    fi
    echo Please tag your new version before releasing with : git tag $VERSION
    exit 64
fi

# MAYBE we dont need this if we have another branch or a third party release ?
#doing bloom release
#if [ "$1" = "pypi" ]; then
#    bloom-release pyros --track indigo --rosdistro indigo
#fi

#uploading package and pushing tags
python setup.py sdist upload -r $1

#TODO : documentation release

