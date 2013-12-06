
from nupic.encoders.multi import MultiEncoder
from nupic.encoders.base import Encoder
import numpy
from nupic.encoders.scalar import ScalarEncoder
from nupic.encoders.vector import VectorEncoder

def _thisIsFunction():
  """just a helper for type comparisons, we need instance of a function"""
  pass

class UtilityEncoder(MultiEncoder):
  """UtilityEncoder act transparently; use input and apply it to encoder, 
     in addition, provide a new field utility (aka \'usefulness\'/goodness/fitness/evaluation) of the input; 
     for use in OPF, provide forced=True and make necessary steps manualy; see below."""
  
  def __init__(self, inputEncoder, utilityEncoder, feval=None, feedbackDelay=0, name=None, forced=False):
    """param inputEncoder: original encoder, accepts input; 
       param feval: an evaluation function: must handle all inputs for inputEncoder, its output must be acceptable by utilityEncoder; util=feval(input);
       param utilityEncoder: encoder, maps output of feval to some values. Must take all outputs of feval;
       param feedbackDelay: (int) #of steps input sensors/actions are away from score default 0), that means:
                 if set to 1, current score is linked with state&action taken 1-step ago; eg: now score(dry)=100 links with state(raining) and action(open umbrella);
       param forced: used to allow this encoder in OPF description.py (set input/utilityEncoder to None), some steps need to be applied by hand (default False)"""

    if (not forced) and not(isinstance(inputEncoder,Encoder)  and isinstance(utilityEncoder, Encoder)):
      raise Exception("must provide an encoder and a function that takes encoder's output and transforms to utility encoder's input")
    if name == "utility":
      raise Exception("name: \'utility\' is reserved for the utility field. Use some other name.")

    super(UtilityEncoder,self).__init__()

    if not forced:
      self.addEncoder(name, inputEncoder)
      self.addEncoder('utility', utilityEncoder)
    
      self._utility = utilityEncoder
      self._encoder = inputEncoder
      self._offset = self._encoder.getWidth()
    else:
      print "WARNING: UtilityEncoder constructed from OPF - initialization incomplete! You need to take care of some steps to make it work!"

    self.setEvaluationFn(feval)
    self._parent = super(UtilityEncoder, self)
    self._name = name
    self.delay = feedbackDelay
    self._bufferedState = [] # list of prev states #TODO: make work for arbitrary n-back (now fixed for 0/1) 

  def encodeIntoArray(self, input, output):
    """on the original input, first compute the utility and then append it as "input" for encoding; 
       the feval function is applied before any encoding"""
    input = self._handleNBack(input)
    score = self.getScoreIN(input)
    encoded_in = self._encoder.encode(input)
    encoded_score = self._utility.encode(score)
    #print "enc_IN=", encoded_in
    #print "enc_SC=", encoded_score
    merged = numpy.concatenate( (encoded_in, encoded_score) )
    #print "enc_merge=", merged
    output[:] = merged
    return output

  def getScoreIN(self, input):
    """compute score of the feedforward input"""
    if self.evaluate is not None:
      return self.evaluate(input)
    else:
      return None


  def getScoreOUT(self, encoded):
    """get the score from the encoded representation"""
    # get the score's portion of data
    score_bits = encoded[self._offset:self.getWidth()]
    dec = self._utility.topDownCompute(score_bits)
    scores = []
    #print "score bits=", score_bits
    #print "dec=",dec
    for re in dec: # case where SDR returned more scores (mixure of SDRs probably)
      scores.append(re.value)
    return scores

  def getData(self, decoded):
    """extract the data part of the decode()'s output"""
    fieldName = decoded[1][0]
    return decoded[0][fieldName][0]

  def setEvaluationFn(self, feval):
    """set the feval function;
       an evaluation function: must handle all inputs from inputEncoder, 
       its output must be acceptable by utilityEncoder; util=feval(input)"""
    if not(type(feval)==type(_thisIsFunction) or feval is None):
      raise Exception("feval must be a function (or None for disabled)")
    self.evaluate=feval

  def _handleNBack(self,state):
    """handle the possible delay between score & state; 
       use buffer of inputs for that
       param state: the input state to be remembered (buffered)"""
    if(self.delay==0): #nothing changes
      return state
    elif(self.delay==1):
      tmp = self._bufferedState[0]
      self._bufferedState[0] = state
      return tmp
    else:
      raise Exception("NBack >1 not yet implemented")
       


######################################################
class SimpleUtilityEncoder(UtilityEncoder):
  """simple version of utility; 
  data is a vector of numbers; defaults=5elements, -5..5, resolution=1;
  utility is a scalar, 0..100, resolution 1"""

  def __init__(self, length=5, feedbackDelay=0, minval=-5, maxval=5, resolution=1, scoreMin=0, scoreMax=100, scoreResolution=1, forced=False):
    dataS = ScalarEncoder(21, minval, maxval, resolution=resolution, name='idx')
    dataV = VectorEncoder(length, dataS, name='data')
    scoreS = ScalarEncoder(21, scoreMin, scoreMax, resolution=scoreResolution, name='utility')
    super(SimpleUtilityEncoder, self).__init__(dataV, scoreS, feedbackDelay=feedbackDelay, name='simpleUtility', forced=forced)
    print "WARNING: feval not set! do not forget to def(ine) the function and set it with setEvaluationFn() "

