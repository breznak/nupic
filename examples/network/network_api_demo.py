#!/usr/bin/env python
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

import copy
import csv
import json
import os

from nupic.algorithms.anomaly import computeAnomalyScore
from nupic.data.datasethelpers import findDataset
from nupic.data.file_record_stream import FileRecordStream
from nupic.engine import Network
from nupic.encoders import MultiEncoder

_VERBOSITY = 0         # how chatty the demo should be
_SEED = 1956             # the random seed used throughout
_DATA_PATH = "extra/hotgym/rec-center-hourly.csv"
_OUTPUT_PATH = "test_output.csv"
_NUM_RECORDS = 2000

# Config field for SPRegion
SP_PARAMS = {
    "spVerbosity": _VERBOSITY,
    "spatialImp": "cpp",
    "globalInhibition": 1,
    "columnCount": 2048,
    "inputWidth": 0,
    "numActiveColumnsPerInhArea": 40,
    "seed": 1956,
    "potentialPct": 0.8,
    "synPermConnected": 0.1,
    "synPermActiveInc": 0.0001,
    "synPermInactiveDec": 0.0005,
    "maxBoost": 1.0,
}

# Config field for TPRegion
TP_PARAMS = {
    "verbosity": _VERBOSITY,
    "columnCount": 2048,
    "cellsPerColumn": 32,
    "inputWidth": 2048,
    "seed": 1960,
    "temporalImp": 'cpp',
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



def createEncoder():
  """Create the encoder instance for our test and return it."""
  encoder = MultiEncoder()
  encoder.addMultipleEncoders({
      "consumption": {
          "clipInput": True,
          "fieldname": u"consumption",
          "maxval": 100.0,
          "minval": 0.0,
          "n": 50,
          "name": u"consumption",
          "type": "ScalarEncoder",
          "w": 21,
      },
      "timestamp_timeOfDay": {
          "fieldname": u"timestamp",
          "name": u"timestamp_timeOfDay",
          "timeOfDay": (21, 9.5),
          "type": "DateEncoder",
      },
      #"timestamp_weekend": {
      #    "fieldname": u"timestamp",
      #    "name": u"timestamp_weekend",
      #    "type": "DateEncoder",
      #    "weekend": 21,
      #},
  })

  return encoder



def createNetwork(dataSource):
  """Create the Network instance.

  :param dataSource: a RecordStream instance to get data from
  :returns: a Network instance ready to run
  """
  encoder = createEncoder()

  network = Network()

  # Our input is sensor data from the gym.csv file
  network.addRegion("sensor", "py.RecordSensor",
                    json.dumps({"verbosity": _VERBOSITY}))
  sensor = network.regions["sensor"].getSelf()
  sensor.encoder = encoder
  sensor.dataSource = dataSource

  # |sensor| -> |spatialPoolerRegion|
  # Create the spatial pooler region
  SP_PARAMS["inputWidth"] = encoder.getWidth()
  network.addRegion("spatialPoolerRegion", "py.SPRegion", json.dumps(SP_PARAMS))

  # Link the SP region to the sensor input
  network.link("sensor", "spatialPoolerRegion", "UniformLink", "")
  network.link("sensor", "spatialPoolerRegion", "UniformLink", "",
               srcOutput="resetOut", destInput="resetIn")
  network.link("spatialPoolerRegion", "sensor", "UniformLink", "",
               srcOutput="spatialTopDownOut", destInput="spatialTopDownIn")
  network.link("spatialPoolerRegion", "sensor", "UniformLink", "",
               srcOutput="temporalTopDownOut", destInput="temporalTopDownIn")

  # |sensor| -> |spatialPoolerRegion| -> |temporalPoolerRegion|
  # Add the Temporal Pooler Region on top of the existing network
  TP_PARAMS["inputWidth"] = SP_PARAMS["columnCount"]
  network.addRegion("temporalPoolerRegion", "py.TPRegion", json.dumps(TP_PARAMS))

  network.link("spatialPoolerRegion", "temporalPoolerRegion", "UniformLink", "")
  network.link("temporalPoolerRegion", "spatialPoolerRegion", "UniformLink", "",
               srcOutput="topDownOut", destInput="topDownIn")
  network.link("sensor", "temporalPoolerRegion", "UniformLink", "",
               srcOutput="resetOut", destInput="resetIn")

  network.initialize()

  spatialPoolerRegion = network.regions["spatialPoolerRegion"]
  spatialPoolerRegion.setParameter("topDownMode", False)
  spatialPoolerRegion.setParameter("learningMode", True)
  spatialPoolerRegion.setParameter("inferenceMode", True)
  spatialPoolerRegion.setParameter("anomalyMode", False)

  temporalPoolerRegion = network.regions["temporalPoolerRegion"]
  temporalPoolerRegion.setParameter("learningMode", True)
  temporalPoolerRegion.setParameter("inferenceMode", True)
  temporalPoolerRegion.setParameter("anomalyMode", True)
  temporalPoolerRegion.setParameter("topDownMode", True)

  return network


def runNetwork(network, writer):
  """Run the network and write output to writer.

  :param network: a Network instance to run
  :param writer: a csv.writer instance to write output to
  """
  sensorRegion = network.regions["sensor"]
  spatialPoolerRegion = network.regions["spatialPoolerRegion"]
  temporalPoolerRegion = network.regions["temporalPoolerRegion"]

  prevPredictedColumns = []

  i = 0
  for _ in xrange(_NUM_RECORDS):
    # Run the network for a single iteration
    network.run(1)

    activeColumns = spatialPoolerRegion.getOutputData(
        "bottomUpOut").nonzero()[0]

    # Calculate the anomaly score using the active columns
    # and previous predicted columns
    anomalyScore = computeAnomalyScore(activeColumns, prevPredictedColumns)

    # Write out the anomaly score along with the record number and consumption
    # value.
    consumption = sensorRegion.getOutputData("sourceOut")[0]
    writer.writerow((i, consumption, anomalyScore))

    # Store the predicted columns for the next timestep
    predictedColumns = temporalPoolerRegion.getOutputData(
        "topDownOut").nonzero()[0]
    prevPredictedColumns = copy.deepcopy(predictedColumns)

    i += 1



if __name__ == "__main__":
  trainFile = findDataset(_DATA_PATH)
  dataSource = FileRecordStream(streamID=trainFile)

  network = createNetwork(dataSource)
  outputPath = os.path.join(os.path.dirname(__file__), _OUTPUT_PATH)
  with open(outputPath, "w") as outputFile:
    writer = csv.writer(outputFile)
    print "Writing output to %s" % outputPath
    runNetwork(network, writer)
