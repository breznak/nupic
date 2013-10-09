#!/usr/bin/env python
# ----------------------------------------------------------------------
# Numenta Platform for Intelligent Computing (NuPIC)
# Copyright (C) 2013, Numenta, Inc.  Unless you have purchased from
# Numenta, Inc. a separate commercial license for this software code, the
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

"""Unit tests for scalar space encoder"""

import unittest2 as unittest

from nupic.encoders.scalarspace import ScalarSpaceEncoder


#########################################################################
class ScalarSpaceEncoderTest(unittest.TestCase):
  '''Unit tests for ScalarSpaceEncoder class'''


  def testScalarSpaceEncoder(self):
    """scalar space encoder"""
    sse = ScalarSpaceEncoder(1,1,2,False,2,1,1,None,0,False,"delta")
    self.assertTrue(sse.isDelta())
    sse = ScalarSpaceEncoder(1,1,2,False,2,1,1,None,0,False,"absolute")
    self.assertFalse(sse.isDelta())

     
###########################################
if __name__ == '__main__':
  unittest.main()
