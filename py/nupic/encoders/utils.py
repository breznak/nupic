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

from array import array

############################################################################
def bitsToString(arr):
  """Returns a string representing a numpy array of 0's and 1's"""
  s = array('c','.'*len(arr))
  for i in xrange(len(arr)):
    if arr[i] == 1:
      s[i]='*'
  return s

############################################################################
def dictToList(dic,nameOrdering=None):
  """convert a dict to list, returns two lists: values, names;
     use: (listVals, listNames) = dictToList(myDict)
     myDict['aa']=1 -> listVals[0]=='1', listNames[0]=='aa' """
  if not isinstance(dic, dict):
    raise Exception("you must provide a dict as a parameter")
  print(nameOrdering)
  listVals=[]
  listNames=[]
  for key in sorted(dic.keys()):
    v=dic[key]
    if isinstance(v,tuple):
      v=v[1] # bloody OPF
    listVals.append(v)
    listNames.append(key)
  return (listVals, listNames)


def listToDict(listValues, listNames=None):
  """convert list of values to a dict, if listNames is not provided, the name
     dict.key "idx{number}" will be used."""
  if not isinstance(listValues,list):
    raise Exception("must give a list as parameter(s)")
  dic = {}
  names=[]
  key=""
  if listNames is not None:
    names=listNames
  else:
    names=range(len(listValues))
    key="idx"
  mx=len(listValues)
  for i in range(0,mx):
    k=key+str(names[i])
    v=listValues[i]
    dic[k] = v
  return dic
#############################################################################

