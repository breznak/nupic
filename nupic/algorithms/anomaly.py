# ----------------------------------------------------------------------
# Numenta Platform for Intelligent Computing (NuPIC)
# Copyright (C) 2014, Numenta, Inc.  Unless you have purchased from
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

"""Anomaly-related algorithms."""

import numpy

from nupic.algorithms.anomaly_likelihood import AnomalyLikelihood
from nupic.common.utils import MovingAverage


def computeRawAnomalyScore(activeColumns, prevPredictedColumns):
  """Computes the raw anomaly score.

  The raw anomaly score is the fraction of active columns not predicted.

  @param activeColumns: array of active column indices
  @param prevPredictedColumns: array of columns indices predicted in prev step
  @return anomaly score 0..1 (float)
  """
  nActiveColumns = len(activeColumns)
  if nActiveColumns > 0:
    # Test whether each element of a 1-D array is also present in a second
    # array. Sum to get the total # of columns that are active and were
    # predicted.
    score = numpy.in1d(activeColumns, prevPredictedColumns).sum()
    # Get the percent of active columns that were NOT predicted, that is
    # our anomaly score.
    score = (nActiveColumns - score) / float(nActiveColumns)
  elif len(prevPredictedColumns) > 0:
    # There were predicted columns but none active.
    score = 1.0
  else:
    # There were no predicted or active columns.
    score = 0.0

  return score



class Anomaly(object):
  """Utility class for generating anomaly scores in different ways.

  Supported modes:
    MODE_PURE - the raw anomaly score as computed by computeRawAnomalyScore
    MODE_LIKELIHOOD - uses the AnomalyLikelihood class on top of the raw
        anomaly scores
    MODE_WEIGHTED - multiplies the likelihood result with the raw anomaly score
        that was used to generate the likelihood
  """


  # anomaly modes supported
  MODE_PURE = "pure"
  MODE_LIKELIHOOD = "likelihood"
  MODE_WEIGHTED = "weighted"
  _supportedModes = (MODE_PURE, MODE_LIKELIHOOD, MODE_WEIGHTED)


  def __init__(self, slidingWindowSize = None, mode=MODE_PURE):
    """
    @param slidingWindowSize (optional) - how many elements are summed up;
        enables moving average on final anomaly score; int >= 0
    @param mode (optional) - (string) how to compute anomaly;
        possible values are:
          - "pure" - the default, how much anomal the value is;
              float 0..1 where 1=totally unexpected
          - "likelihood" - uses the anomaly_likelihood code;
              models probability of receiving this value and anomalyScore
          - "weighted" - "pure" anomaly weighted by "likelihood"
              (anomaly * likelihood)
    """
    self._mode = mode
    if slidingWindowSize > 0:
      self._movingAverage = MovingAverage(windowSize=int(slidingWindowSize))
    else:
      self._movingAverage = None

    # Using cumulative anomaly, sliding window
    if (self._movingAverage is None) and (slidingWindowSize is not None):
      raise TypeError(
          "Anomaly: if you define slidingWindowSize, it has to be an "
          "integer > 0;  slidingWindowSize=%r" % slidingWindowSize)

    if self._mode == Anomaly.MODE_LIKELIHOOD:
      self._likelihood = AnomalyLikelihood() # probabilistic anomaly
    if not self._mode in Anomaly._supportedModes:
      raise ValueError("Invalid anomaly mode; only supported modes are: "
                       "Anomaly.MODE_PURE, Anomaly.MODE_LIKELIHOOD, "
                       "Anomaly.MODE_WEIGHTED; you used: %r" % self._mode)


  def computeAnomalyScore(self, activeColumns, predictedColumns, value=None,
                          timestamp=None):
    """Compute the anomaly score as the percent of active columns not predicted.

    @param activeColumns: array of active column indices
    @param predictedColumns: array of columns indices predicted in this step
                             (used for anomaly in step T+1)
    @param value: (optional) metric value of current input
                              (used in anomaly-likelihood)
    @param timestamp: (optional) date timestamp when the sample occured
                              (used in anomaly-likelihood)
    @return the computed anomaly score; float 0..1
    """
    # Start by computing the raw anomaly score.
    anomalyScore = computeRawAnomalyScore(activeColumns, predictedColumns)

    # Compute final anomaly based on selected mode.
    if self._mode == Anomaly.MODE_PURE:
      score = anomalyScore
    elif self._mode == Anomaly.MODE_LIKELIHOOD:
      # TODO add tests for likelihood modes
      probability = self._likelihood.anomalyProbability(
          value, anomalyScore, timestamp)
      score = probability
    elif self._mode == Anomaly.MODE_WEIGHTED:
      probability = self._likelihood.anomalyProbability(
          value, anomalyScore, timestamp)
      score = anomalyScore * probability

    # Last, do moving-average if windowSize was specified.
    if self._movingAverage is not None:
      score, _, _ = self._movingAverage.next(score)

    return score
