#!/bin/bash
# ----------------------------------------------------------------------
# Numenta Platform for Intelligent Computing (NuPIC)
# Copyright (C) 2013, Numenta, Inc.  Unless you have an agreement
# with Numenta, Inc., for a separate license for this software code, the
# following terms and conditions apply:
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see http://www.gnu.org/licenses.
#
# http://numenta.org/licenses/
# ----------------------------------------------------------------------

echo
echo Running `basename $0`...
echo

pip install python-coveralls
pushd ${TRAVIS_BUILD_DIR}
coveralls -i
popd

if [ $PY_VER = '2.7' ] && [ $CC = 'clang' ]; then pip install python-coveralls; fi

# Only publishing unit test coverage at this point.
if [ $PY_VER = '2.7' ] && [ $CC = 'clang' ]; then coveralls -i --data_file=.coverage_unit; fi

# TODO: figure out how to publish integration test coverage as well.
