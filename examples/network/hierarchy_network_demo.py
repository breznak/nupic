#!/usr/bin/env python
# ----------------------------------------------------------------------
# Numenta Platform for Intelligent Computing (NuPIC)
# Copyright (C) 2015, Numenta, Inc.  Unless you have an agreement
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

"""
An example of a hierarchy of cortical regions in a Network.
"""

import copy
import csv
import json
import os

from nupic.algorithms.anomaly import computeRawAnomalyScore
from nupic.data.datasethelpers import findDataset
from nupic.data.file_record_stream import FileRecordStream
from nupic.engine import Network
from nupic.encoders import MultiEncoder

# Level of detail of console output. Int value from 0 (none)
# to 3 (super detailed)
_VERBOSITY = 0

# Seed used for random number generation
_SEED = 2045
_INPUT_FILE_PATH = "../prediction/data/extra/hotgym/rec-center-hourly.csv"
_OUTPUT_FILE_NAME = "hierarchy-demo-output.csv"

# TODO Determine this from the FileRecordStream somehow?
_NUM_INPUT_RECORDS = 20
# _NUM_INPUT_RECORDS = 2000

# Parameter dict for SPRegion
SP_PARAMS = {
    "spVerbosity": _VERBOSITY,
    "spatialImp": "cpp",
    "seed": _SEED,

    # determined and set during network creation
    "inputWidth": 0,

    # @see nupic.research.spatial_pooler.SpatialPooler for explanations
    "globalInhibition": 1,
    "columnCount": 2048,
    "numActiveColumnsPerInhArea": 40,
    "potentialPct": 0.8,
    "synPermConnected": 0.1,
    "synPermActiveInc": 0.0001,
    "synPermInactiveDec": 0.0005,
    "maxBoost": 1.0,
}

# Parameter dict for TPRegion
TP_PARAMS = {
    "verbosity": _VERBOSITY,
    "temporalImp": "cpp",
    "seed": _SEED,

    # @see nupic.research.temporal_memory.TemporalMemory for explanations
    # TODO some of these params are not found there
    "columnCount": 2048,
    "cellsPerColumn": 32,
    "inputWidth": 2048,
    "newSynapseCount": 20,
    "maxSynapsesPerSegment": 32,
    "maxSegmentsPerCell": 128,
    "initialPerm": 0.21,
    "permanenceInc": 0.1,
    "permanenceDec": 0.1,
    "globalDecay": 0.0,
    "maxAge": 0,
    "minThreshold": 9,
    "activationThreshold": 12,
    "outputType": "normal",
    "pamLength": 3,
}

_RECORD_SENSOR = "sensorRegion"
_L1_SPATIAL_POOLER = "l1spatialPoolerRegion"
_L1_TEMPORAL_POOLER = "l1temporalPoolerRegion"
_L1_CLASSIFIER = "l1classifier"

_L2_SPATIAL_POOLER = "l2spatialPoolerRegion"
_L2_TEMPORAL_POOLER = "l2temporalPoolerRegion"

def createEncoder():
  """
  Creates and returns a #MultiEncoder including a ScalarEncoder for
  energy consumption and a DateEncoder for the time of the day.

  @see nupic/encoders/__init__.py for type to file-name mapping
  @see nupic/encoders for encoder source files
  """
  encoder = MultiEncoder()
  encoder.addMultipleEncoders({
      "consumption": {
          "fieldname": u"consumption",
          "type": "ScalarEncoder",
          "name": u"consumption",
          "minval": 0.0,
          "maxval": 100.0,
          "clipInput": True,
          "w": 21,
          "n": 50,
      },
      "timestamp_timeOfDay": {
          "fieldname": u"timestamp",
          "type": "DateEncoder",
          "name": u"timestamp_timeOfDay",
          "timeOfDay": (21, 9.5),
      },
  })
  return encoder


def createRecordSensor(network, name, dataSource):
  """
  Creates a RecordSensor region that allows us to specify a file record
  stream as the input source.
  """

  # Specific type of region. Possible options can be found in /nupic/regions/
  regionType = "py.RecordSensor"

  # Creates a json from specified dictionary.
  regionParams = json.dumps({"verbosity": _VERBOSITY})
  network.addRegion(name, regionType, regionParams)

  # getSelf returns the actual region, instead of a region wrapper
  sensorRegion = network.regions[name].getSelf()

  # Specify how RecordSensor encodes input values
  sensorRegion.encoder = createEncoder()

  # Specify the dataSource as a file record stream instance
  sensorRegion.dataSource = dataSource
  return sensorRegion


def createSpatialPooler(network, name, inputWidth):
  # Create the spatial pooler region
  SP_PARAMS["inputWidth"] = inputWidth
  spatialPoolerRegion = network.addRegion(name, "py.SPRegion",
                                          json.dumps(SP_PARAMS))
  # Make sure learning is enabled
  spatialPoolerRegion.setParameter("learningMode", True)
  # We want temporal anomalies so disable anomalyMode in the SP. This mode is
  # used for computing anomalies in a non-temporal model.
  spatialPoolerRegion.setParameter("anomalyMode", False)
  return spatialPoolerRegion


def createTemporalPooler(network, name):
  temporalPoolerRegion = network.addRegion(name,
                                           "py.TPRegion",
                                           json.dumps(TP_PARAMS))
  # Enable topDownMode to get the predicted columns output
  temporalPoolerRegion.setParameter("topDownMode", True)
  # Make sure learning is enabled (this is the default)
  temporalPoolerRegion.setParameter("learningMode", True)
  # Enable inference mode so we get predictions
  temporalPoolerRegion.setParameter("inferenceMode", True)
  # Enable anomalyMode to compute the anomaly score. This actually doesn't work
  # now so doesn't matter. We instead compute the anomaly score based on
  # topDownOut (predicted columns) and SP bottomUpOut (active columns).
  temporalPoolerRegion.setParameter("anomalyMode", True)
  return temporalPoolerRegion


def createNetwork(dataSource):
  """Creates and returns a new Network with a sensor region reading data from
  'dataSource'.
  TODO describe hierarchy
  @param dataSource - A RecordStream containing the input data
  @returns a Network ready to run
  """
  network = Network()

  # Create and add a record sensor and a SP region
  sensor = createRecordSensor(network, name=_RECORD_SENSOR,
                              dataSource=dataSource)
  createSpatialPooler(network, name=_L1_SPATIAL_POOLER,
                      inputWidth=sensor.encoder.getWidth())

  # Link the SP region to the sensor input
  linkType = "UniformLink"
  linkParams = ""
  network.link(_RECORD_SENSOR, _L1_SPATIAL_POOLER, linkType, linkParams)
  # TODO remove
  # network.link(_RECORD_SENSOR, _L1_SPATIAL_POOLER, linkType, linkParams,
  #              srcOutput="resetOut", destInput="resetIn")
  # network.link(_L1_SPATIAL_POOLER, _RECORD_SENSOR, linkType, linkParams,
  #              srcOutput="spatialTopDownOut", destInput="spatialTopDownIn")
  # network.link(_L1_SPATIAL_POOLER, _RECORD_SENSOR, linkType, "",
  #              srcOutput="temporalTopDownOut", destInput="temporalTopDownIn")

  # Create and add a TP region
  TP_PARAMS["cellsPerColumn"] = 12
  l1temporalPooler = createTemporalPooler(network, _L1_TEMPORAL_POOLER)

  # Link SP region to TP region in FF direction
  network.link(_L1_SPATIAL_POOLER, _L1_TEMPORAL_POOLER, linkType, linkParams)
  # TODO remove
  # network.link(_L1_TEMPORAL_POOLER, _L1_SPATIAL_POOLER, linkType, linkParams,
  #              srcOutput="topDownOut", destInput="topDownIn")

  # Add a classifier
  # @see clamodel.py
  # TODO mention params and give defaults
  clParams = {}
  network.addRegion(_L1_CLASSIFIER, "py.CLAClassifierRegion",
                    json.dumps(clParams))
  network.link(_RECORD_SENSOR, _L1_CLASSIFIER, "UniformLink", "",
               srcOutput="categoryOut", destInput="categoryIn")
  # TODO Mention srcOutput="bottomUpOut" destInput="bottomUpIn"
  network.link(_L1_TEMPORAL_POOLER, _L1_CLASSIFIER, "UniformLink", "")

  # Second Level
  l2inputWidth = l1temporalPooler.getSelf().getOutputElementCount("bottomUpOut")
  createSpatialPooler(network, name=_L2_SPATIAL_POOLER, inputWidth=l2inputWidth)
  network.link(_L1_TEMPORAL_POOLER, _L2_SPATIAL_POOLER, linkType, linkParams)

  createTemporalPooler(network, _L2_TEMPORAL_POOLER)
  network.link(_L2_SPATIAL_POOLER, _L2_TEMPORAL_POOLER, linkType, linkParams)
  return network


def runNetwork(network, writer):
  """
  Runs specified Network writing the ensuing anomaly score to writer.

  @param network: The Network instance to be run
  @param writer: A csv.writer used to write to output file.
  """
  sensorRegion = network.regions[_RECORD_SENSOR]
  l1SpRegion = network.regions[_L1_SPATIAL_POOLER]
  l1TpRegion = network.regions[_L1_TEMPORAL_POOLER]
  l1Classifier = network.regions[_L1_CLASSIFIER]

  l2SpRegion = network.regions[_L2_SPATIAL_POOLER]
  l2TpRegion = network.regions[_L2_TEMPORAL_POOLER]
  # TODO Print something out from L2?

  iterations = 1
  prevPredictedColumns = []
  for i in xrange(_NUM_INPUT_RECORDS):
    # Run the network for a single iteration
    network.run(iterations)

    # recordNum, patternNZ, classification
    # l1Classifier.getSelf().customCompute(recordNum=i, patternNZ=,
    #                                      classification=)
    # Get
    # customCompute(self, recordNum, patternNZ, classification)

    # nonzero() returns the indices of the elements that are non-zero,
    # here the elements are the indices of the active columns
    activeColumns = l1SpRegion.getOutputData("bottomUpOut").nonzero()[0]

    # Calculate the anomaly score using the active columns
    # and previous predicted columns
    anomalyScore = computeRawAnomalyScore(activeColumns, prevPredictedColumns)

    # Write out the anomaly score along with the record number and consumption
    # value.
    consumption = sensorRegion.getOutputData("sourceOut")[0]
    writer.writerow((i, consumption, anomalyScore))

    # Store the predicted columns for the next timestep
    predictedColumns = l1TpRegion.getOutputData("topDownOut").nonzero()[0]
    prevPredictedColumns = copy.deepcopy(predictedColumns)


def runDemo():
  trainFile = findDataset(_INPUT_FILE_PATH)
  dataSource = FileRecordStream(streamID=trainFile)
  network = createNetwork(dataSource)
  outputPath = os.path.join(os.path.dirname(__file__), _OUTPUT_FILE_NAME)
  with open(outputPath, "w") as outputFile:
    writer = csv.writer(outputFile)
    print "Writing output to: %s" % outputPath
    runNetwork(network, writer)


if __name__ == "__main__":
  runDemo()
