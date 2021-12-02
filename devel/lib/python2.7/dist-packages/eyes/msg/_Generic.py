# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from eyes/Generic.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Generic(genpy.Message):
  _md5sum = "7a33c669a022f7fea29ccba33d517b1f"
  _type = "eyes/Generic"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """char identifier
bool left_forward
bool right_forward
uint8 left_speed
uint8 right_speed
bool timed
uint32 duration
"""
  __slots__ = ['identifier','left_forward','right_forward','left_speed','right_speed','timed','duration']
  _slot_types = ['char','bool','bool','uint8','uint8','bool','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       identifier,left_forward,right_forward,left_speed,right_speed,timed,duration

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Generic, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.identifier is None:
        self.identifier = 0
      if self.left_forward is None:
        self.left_forward = False
      if self.right_forward is None:
        self.right_forward = False
      if self.left_speed is None:
        self.left_speed = 0
      if self.right_speed is None:
        self.right_speed = 0
      if self.timed is None:
        self.timed = False
      if self.duration is None:
        self.duration = 0
    else:
      self.identifier = 0
      self.left_forward = False
      self.right_forward = False
      self.left_speed = 0
      self.right_speed = 0
      self.timed = False
      self.duration = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6BI().pack(_x.identifier, _x.left_forward, _x.right_forward, _x.left_speed, _x.right_speed, _x.timed, _x.duration))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.identifier, _x.left_forward, _x.right_forward, _x.left_speed, _x.right_speed, _x.timed, _x.duration,) = _get_struct_6BI().unpack(str[start:end])
      self.left_forward = bool(self.left_forward)
      self.right_forward = bool(self.right_forward)
      self.timed = bool(self.timed)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6BI().pack(_x.identifier, _x.left_forward, _x.right_forward, _x.left_speed, _x.right_speed, _x.timed, _x.duration))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.identifier, _x.left_forward, _x.right_forward, _x.left_speed, _x.right_speed, _x.timed, _x.duration,) = _get_struct_6BI().unpack(str[start:end])
      self.left_forward = bool(self.left_forward)
      self.right_forward = bool(self.right_forward)
      self.timed = bool(self.timed)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6BI = None
def _get_struct_6BI():
    global _struct_6BI
    if _struct_6BI is None:
        _struct_6BI = struct.Struct("<6BI")
    return _struct_6BI
