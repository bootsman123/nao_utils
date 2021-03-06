"""autogenerated by genmsg_py from BumperButton.msg. Do not edit."""
import roslib.message
import struct


class BumperButton(roslib.message.Message):
  _md5sum = "c03def6f70a334da80436f2b892826ab"
  _type = "nao_sensors/BumperButton"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Constants.
uint8 BUTTON_RIGHT = 1
uint8 BUTTON_LEFT = 2

uint8 STATE_RELEASED = 0
uint8 STATE_PRESSED = 1

# Fields.
uint8 button
uint8 state


"""
  # Pseudo-constants
  BUTTON_RIGHT = 1
  BUTTON_LEFT = 2
  STATE_RELEASED = 0
  STATE_PRESSED = 1

  __slots__ = ['button','state']
  _slot_types = ['uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       button,state
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(BumperButton, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.button is None:
        self.button = 0
      if self.state is None:
        self.state = 0
    else:
      self.button = 0
      self.state = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2B.pack(_x.button, _x.state))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.button, _x.state,) = _struct_2B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2B.pack(_x.button, _x.state))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.button, _x.state,) = _struct_2B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2B = struct.Struct("<2B")
