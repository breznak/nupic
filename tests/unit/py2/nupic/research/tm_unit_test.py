#!/usr/bin/env python
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

import unittest

from nupic.research.TM import Connections, TM



class TMTest(unittest.TestCase):


  def testInit(self):
    pass



class ConnectionsTest(unittest.TestCase):


  def testInit(self):
    columnDimensions = [2048]
    cellsPerColumn = 32

    connections = Connections(columnDimensions, cellsPerColumn)
    self.assertEqual(connections.columnDimensions, columnDimensions)
    self.assertEqual(connections.cellsPerColumn, cellsPerColumn)


  def testInitInvalidParams(self):
    # Invalid columnDimensions
    args = [[], 32]
    self.assertRaises(ValueError, Connections, *args)

    # Invalid cellsPerColumn
    args = [[2048], 0]
    self.assertRaises(ValueError, Connections, *args)
    args = [[2048], -10]
    self.assertRaises(ValueError, Connections, *args)


  def testCellsForColumn1D(self):
    connections = Connections([2048], 5)
    expectedCells = {5, 6, 7, 8, 9}
    self.assertEqual(connections.cellsForColumn(1), expectedCells)


  def testCellsForColumn2D(self):
    connections = Connections([64, 64], 4)
    expectedCells = {256, 257, 258, 259}
    self.assertEqual(connections.cellsForColumn(64), expectedCells)


  def testCellsForColumnInvalidColumn(self):
    connections = Connections([64, 64], 4)

    try:
      connections.cellsForColumn(4095)
    except IndexError:
      self.fail("IndexError raised unexpectedly")

    args = [connections, 4096]
    self.assertRaises(IndexError, Connections.cellsForColumn, *args)

    args = [connections, -1]
    self.assertRaises(IndexError, Connections.cellsForColumn, *args)


  def testColumnForCell1D(self):
    connections = Connections([2048], 5)
    self.assertEqual(connections.columnForCell(0), 0)
    self.assertEqual(connections.columnForCell(4), 0)
    self.assertEqual(connections.columnForCell(5), 1)
    self.assertEqual(connections.columnForCell(10239), 2047)


  def testColumnForCell2D(self):
    connections = Connections([64, 64], 4)
    self.assertEqual(connections.columnForCell(0), 0)
    self.assertEqual(connections.columnForCell(3), 0)
    self.assertEqual(connections.columnForCell(4), 1)
    self.assertEqual(connections.columnForCell(16383), 4095)


  def testColumnForCellInvalidCell(self):
    connections = Connections([64, 64], 4)

    try:
      connections.columnForCell(16383)
    except IndexError:
      self.fail("IndexError raised unexpectedly")

    args = [connections, 16384]
    self.assertRaises(IndexError, Connections.columnForCell, *args)

    args = [connections, -1]
    self.assertRaises(IndexError, Connections.columnForCell, *args)



if __name__ == '__main__':
  unittest.main()
