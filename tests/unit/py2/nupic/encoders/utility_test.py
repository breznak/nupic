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


import unittest2 as unittest


from nupic.encoders.scalar import ScalarEncoder
from nupic.encoders.vector import VectorEncoder
from nupic.encoders.utility import UtilityEncoder

class UtilityEncoderTest(unittest.TestCase):
  """testing Utility encoder"""

  def setUp(self):
    self.data = [1,2,3]

    # encoder for score: 0..100, fine-grained to 0.5
    self.scoreEnc = ScalarEncoder(3, 0, 100, resolution=0.5, name='score')

    # encoder for the input (data) part
    elem = ScalarEncoder(1,0,3,resolution=1)
    self.dataEnc = VectorEncoder(len(self.data), elem, typeCastFn=int, name='data')

    # utility encoder
    def sumAll(list):
      return sum(list)

    self.fn = sumAll

    self.utilityEnc = None

  def testInitialization(self):
    """creating a utility encoder"""
    util = UtilityEncoder(self.dataEnc, self.scoreEnc, feval=self.fn, name='starter')
    assert True==isinstance(util, UtilityEncoder)

##########################################################
if __name__ == '__main__':
  unittest.main()
