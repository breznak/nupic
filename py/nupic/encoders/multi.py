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

from base import Encoder
from nupic.encoders.scalar import ScalarEncoder
from nupic.encoders.adaptivescalar import AdaptiveScalarEncoder
from nupic.encoders.date import DateEncoder
from nupic.encoders.logenc import LogEncoder
from nupic.encoders.category import CategoryEncoder
from nupic.encoders.sdrcategory import SDRCategoryEncoder
from nupic.encoders.sdrrandom import SDRRandomEncoder
from nupic.encoders.nonuniformscalar import NonUniformScalarEncoder
from nupic.encoders.delta import DeltaEncoder
from nupic.encoders.scalarspace import ScalarSpaceEncoder
# multiencoder must be imported last because it imports * from this module!
from nupic.encoders.utils import bitsToString 

from nupic.data import dictutils
from nupic.data.dictutils import DictObj
from nupic.encoders.utils import listToDict, dictToList
import numpy

class MultiEncoder(Encoder):
  """A MultiEncoder encodes a dictionary or object with
  multiple components. A MultiEncode contains a number
  of sub-encoders, each of which encodes a separate component.

  input is overloaded to accept any of: DictObj, list; 
  output can be in these formats too, use self.outputMode to set desired one."""


  def __init__(self, encoderDescriptions=None, outputMode='Dict'):
    """constructs MultiEncoder; 
     params: (optional) encoderDescriptions - add these encoders; 
     params: (optional) outputMode - one of {'Dict','List'} specify type of output"""
    self.width = 0
    self.encoders = []
    self.description = []
    self.name = 'MultiEncoder'
    if encoderDescriptions is not None:
      self.addMultipleEncoders(encoderDescriptions)
    self.outputMode = outputMode
    self._encodersNames=set([])

  ############################################################################
  def decode(self, encoded, ff=''):
    """decode (encoded) SDR, output depends on value of self.outputMode: 
     'Dict' - standard behavior, return a dict (DictObj)
     'List' - return python's list"""
    result = super(MultiEncoder, self).decode(encoded, ff)
    print(result)
    if self.outputMode == 'List':
      result = dictToList(result[0],result[1])
    return result
     
  ############################################################################
  def setFieldStats(self, fieldName, fieldStatistics ):
    for (name, encoder, offset) in self.encoders:
      encoder.setFieldStats(name, fieldStatistics)
      
  ############################################################################
  def addEncoder(self, fieldName, encoder):
    """ add encoder to the pool, 
      fieldName is the variable this encoder will cover, encoder is the encoder instance;
      Note: 'List's do not! use name-variable pairs, so variable representation depends on order when added."""
    if fieldName in self._encodersNames:
      raise Exception("Only one encoder for same field possible! %s" % fieldName)
    self._encodersNames.add(fieldName)
    self.encoders.append((fieldName, encoder, self.width))
    for d in encoder.getDescription():
      self.description.append((d[0], d[1] + self.width)) # (name,offset)
    self.width += encoder.getWidth()

    self._flattenedEncoderList = None
    self._flattenedFieldTypeList = None

  ############################################################################
  # overloaded function encodeIntoArray, calls specific encodeIntoArray_* as needed
  def encodeIntoArray(self, obj, output):
    """encode, 
     accepts any of: dict, list
     returns SDR"""
    if(isinstance(obj, list)):
      if not len(obj)==len(self.encoders):
        raise Exception("obj must be specified and must be a list of length == self.encoders (%d)" % len(self.encoders))
      obj = listToDict(obj, list(self._encodersNames))
    elif(isinstance(obj,dict)):
      pass
    else:
      raise Exception("obj type must be one of: list, dict")
      
    for name, encoder, offset in self.encoders:
        encoder.encode(obj[name], output[offset:])

  ############################################################################
  def getDescription(self):
    return self.description

  ############################################################################
  def getWidth(self):
    return self.width

  def setLearning(self,learningEnabled):
    encoders = self.getEncoderList()
    for encoder in encoders:
      encoder.setLearning(learningEnabled)
    return

  ############################################################################
  def encodeField(self, fieldName, value):
    for (name, encoder, _) in self.encoders:
      if name == fieldName:
        return encoder.encode(value)

  ############################################################################
  def encodeEachField(self, inputRecord):
    encodings = []
    print("inrec %s" % (inputRecord))
    assert False
    for (name, encoder, _) in self.encoders:
      encodings.append(encoder.encode(getattr(inputRecord, name)))
    return encodings

  ############################################################################
  def addMultipleEncoders(self, fieldEncodings):
    """
    fieldEncodings -- a dict of dicts, mapping field names to the field params
                        dict. 

    Each field params dict has the following keys
    1) data fieldname that matches the key ('fieldname')
    2) an encoder type ('type')
    3) and the encoder params (all other keys)

    For example,
    fieldEncodings={
        'dateTime': dict(fieldname='dateTime', type='DateEncoder',
                         timeOfDay=(5,5)),
        'attendeeCount': dict(fieldname='attendeeCount', type='ScalarEncoder',
                              name='attendeeCount', minval=0, maxval=250,
                              clipInput=True, w=5, resolution=10),
        'consumption': dict(fieldname='consumption',type='ScalarEncoder',
                            name='consumption', minval=0,maxval=110,
                            clipInput=True, w=5, resolution=5),
    }
    """
    
    # do not sort, order matters
    encoderList = fieldEncodings.items()
    for key, fieldParams in encoderList:
      if ':' not in key and fieldParams is not None:
        fieldParams = fieldParams.copy()
        fieldName   = fieldParams.pop('fieldname')
        encoderName = fieldParams.pop('type')
        try:
          self.addEncoder(fieldName, eval(encoderName)(**fieldParams))
        except TypeError, e:
          print ("#### Error in constructing %s encoder. Possibly missing "
                "some required constructor parameters. Parameters "
                "that were provided are: %s" %  (encoderName, fieldParams))
          raise


################################################################################
class SimpleVector(MultiEncoder): 
  """SimpleVector represents an array/vector of numbers; 
   represented by ScalarEncoders, it's a convenience wrapper around MultiEncoder, 
   output is as numpy array, so it can be used for other processing."""
   
  def __init__(self, length, minVal, maxVal, outputMode='List', w=7, verbosity=0, fieldNames=None):
    """param length: how many values/numbers the array holds; array.size()
       param minVal, maxVal : range of all values
       param (opt) outputMode: see MultiEncoder, output type
       param (opt) w: how many onbits represent a value, see ScalarEncoder
       param (opt) resolution: w>resolution, two patterns are considered diff values when their overlap < resolution, see ScalarEncoder
       param (opt) verbosity
       param (opt) fieldNames: list of strings, must be same size as values == length == #encoders, provide name for i-th value
    """

    super(SimpleVector, self).__init__(None, outputMode)
 
    if fieldNames is not None and len(fieldNames)!=length:
      raise Exception("if fieldNames is specified, it must be a list of size == length")

    for i in range(0,length):
      name = "idx" + str(i)
      if(fieldNames is not None):
        name = fieldNames[i]
      self.addEncoder(name, ScalarEncoder(w, minVal, maxVal, resolution=1, name=name))
