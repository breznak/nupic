#!/usr/bin/env python
# ----------------------------------------------------------------------
# Chetan Surpur
# Copyright (C) 2013
#
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

"""A client to create a CLA model for Linguist."""

import sys
from nupic.frameworks.opf.modelfactory import ModelFactory
import model_params
import re
import numpy

NUM_REPEATS = 1
PRINT_EVERY_REPEAT_N = 1
TERMINATORS = ['.','!','?','|']

def createModel():
  return ModelFactory.create(model_params.MODEL_PARAMS)

def runLinguist(datapath):
  model = createModel()
  model.enableInference({'predictedField': 'letter'})

  i = 1
  for r in range(NUM_REPEATS):
    for c in xrange(0,100):
        modelInput = {'letter': numpy.random.randint(0,100,1)[0]}
        result = model.run(modelInput)
        print result
        i += 1


  print "test"
  model.disableLearning()
  for i in [32, 32.2, 64, 64.6]:
   modelInput = {'letter': i}
   result = model.run(modelInput)
   c=result.inferences['prediction'][0]
   print result, "input=",i,"result=",c

  return model


if __name__ == "__main__":
    model = runLinguist(None)
