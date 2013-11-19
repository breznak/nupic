
from nupic.encoders.multi import MultiEncoder as ME


def _thisIsFunction():
  """just a helper for type comparisons, we need instance of a function"""
  pass

class UtilityEncoder(MultiEncoder):
  """UtilityEncoder act transparently; use input and apply it to encoder, 
     in addition, provide a new field utility (aka \'usefulness\'/goodness/fitness/evaluation) of the input"""
  
  def __init__(self, inputEncoder, utilityEncoder, feval=None, name='utility'):
    """param inputEncoder: original encoder, accepts input; 
       param feval: an evaluation function: must handle all inputs for inputEncoder, its output must be acceptable by utilityEncoder; util=feval(input);
       param utilityEncoder: encoder, maps output of feval to some values. Must take all outputs of feval"""
    if not(isinstance(encoder,Encoder)  and isinstance(utility, Encoder)):
      raise Exception("must provide an encoder and a function that takes encoder's output and transforms to utility encoder's input")

    super(UtilityEncoder,self).__init__()

    self.addEncoder(name, inputEncoder)
    self.addEncoder('utility', utilityEncoder)
    
    self._utility = utilityEncoder
    self._encoder = inputEncoder
    self.eval = self.setEvaluationFn(feval)
    self._offset = self._encoder.getWidth()
    self._parent = super(UtilityEncoder, self)
    self._name = name

  def encodeIntoArray(self, input, output)
    """on the original input, first compute the utility and then append it as "input" for encoding; 
       the feval function is applied before any encoding"""
    score = self.getScoreIN(input)
    merged_input = [input, score]
    output = self._parent.encode(merged_input)


  def decode(self, encoded, parentFieldNames=''):
    """takes the extended input from above, and recovers back values for orig input and utility"""
    return self._parent.decode(encoded, parentFieldNames)

  
  def getScoreIN(self, input):
    """compute score of the feedforward input"""
    if self.eval is not None:
      return self.eval(input)
    else:
      return None


  def getScoreOUT(self, encoded):
    """get the score from the encoded representation"""
    # get the score's portion of data
    score_bits = encoded[self._offset:self.getWidth()]
    self._utility.topDownCompute(encoded).value 


  def setEvaluationFn(self):
    """set the feval function;
       an evaluation function: must handle all inputs from inputEncoder, 
       its output must be acceptable by utilityEncoder; util=feval(input)"""
    if not(type(feval)==type(_thisIsFunction) or feval is None):
      raise Exception("feval must be a function (or None for disabled)")

    self.eval=feval

