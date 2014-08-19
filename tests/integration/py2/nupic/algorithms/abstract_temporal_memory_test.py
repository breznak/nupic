# ----------------------------------------------------------------------
# Numenta Platform for Intelligent Computing (NuPIC)
# Copyright (C) 2014, Numenta, Inc.  Unless you have an agreement
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

import unittest2 as unittest

from nupic.data.sequence_machine import SequenceMachine
from nupic.test.temporal_memory_test_machine import TemporalMemoryTestMachine
from nupic.research.temporal_memory import TemporalMemory



class AbstractTemporalMemoryTest(unittest.TestCase):

  VERBOSITY = 1

  DEFAULT_TM_PARAMS = {}


  def setUp(self):
    self.tm = None
    self.patternMachine = None
    self.sequenceMachine = None
    self.tmTestMachine = None


  def initTM(self, overrides=None):
    params = self.computeTMParams(overrides)
    self.tm = TemporalMemory(**params)


  def finishSetUp(self, patternMachine):
    self.patternMachine = patternMachine
    self.sequenceMachine = SequenceMachine(self.patternMachine)
    self.tmTestMachine = TemporalMemoryTestMachine(self.tm)


  def feedTM(self, sequence, learn=True, num=1):
    repeatedSequence = sequence * num
    results = self.tmTestMachine.feedSequence(repeatedSequence, learn=learn)

    detailedResults = self.tmTestMachine.computeDetailedResults(
      results,
      repeatedSequence)

    return detailedResults


  # ==============================
  # Helper functions
  # ==============================

  def computeTMParams(self, overrides):
    params = dict(self.DEFAULT_TM_PARAMS)
    params.update(overrides or {})
    return params

