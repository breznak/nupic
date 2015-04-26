# ----------------------------------------------------------------------
# Numenta Platform for Intelligent Computing (NuPIC)
# Copyright (C) 2015, Numenta, Inc.  Unless you have purchased from
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

"""Region for computing the anomaly score."""

import numpy

from nupic.algorithms.anomaly import Anomaly
from nupic.regions.PyRegion import PyRegion



class AnomalyRegion(PyRegion):
  """Region for computing the anomaly score."""


  @classmethod
  def getSpec(cls):
    return {
        "description": ("Region that computes anomaly scores from temporal "
                        "memory."),
        "singleNodeOnly": True,
        "inputs": {
            "activeColumns": {
                "description": "The currently active columns.",
                "regionLevel": True,
                "dataType": "Real32",
                "count": 0,
                "required": True,
                "isDefaultInput": False,
                "requireSplitterMap": False,
            },
            "predictedColumns": {
                "description": "The currently predicted columns.",
                "regionLevel": True,
                "dataType": "Real32",
                "count": 0,
                "required": True,
                "isDefaultInput": False,
                "requireSplitterMap": False,
            },
            "rawInput": {
                "description": "The raw values coming from the input sensor.",
                "regionLevel": True,
                "dataType": "Real32", #TODO what type to set here? can be anything
                "count": 0,
                "required": False,
                "isDefaultInput": False,
                "requireSplitterMap": False,
            },
        },
        "outputs": {
            "rawAnomalyScore": {
                "description": "The raw anomaly score.",
                "dataType": "Real32",
                "count": 1,
                "regionLevel": True,
                "isDefaultOutput": True,
            },
        },
        "parameters": {
        },
        "commands": {
        },
    }


  def __init__(self, *args, **kwargs):
    super(AnomalyRegion, self).__init__(*args, **kwargs)
    print args, type(args)
    windowSize = kwargs.get("slidingWindowSize", None)
    mode = kwargs.get("mode", Anomaly.MODE_PURE)
    binaryThr = kwargs.get("binaryAnomalyThreshold", None)
    self.anomaly = Anomaly(windowSize, mode, binaryThr)

    self.prevPredictedColumns = numpy.zeros([], dtype="float32")


  def initialize(self, inputs, outputs):
    pass


  def compute(self, inputs, outputs):
    activeColumns = inputs["activeColumns"].nonzero()[0]
    rawInput = inputs.get("rawInput", None)
    timestamp = inputs.get("timestamp", None)
    print "RAW=", rawInput

    rawAnomalyScore = self.anomaly.compute(activeColumns, self.prevPredictedColumns,
                                           rawInput, timestamp)

    self.prevPredictedColumns = inputs["predictedColumns"].nonzero()[0]

    outputs["rawAnomalyScore"][0] = rawAnomalyScore
