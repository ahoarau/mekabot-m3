"""autogenerated by genpy from meka_ik/MekaIKRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MekaIKRequest(genpy.Message):
  _md5sum = "27d4170986f50b4c204c4a40684420d7"
  _type = "meka_ik/MekaIKRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string arm_name
float32[3] end_position
float32[3] end_rpy
float32[7] angles_current

"""
  __slots__ = ['arm_name','end_position','end_rpy','angles_current']
  _slot_types = ['string','float32[3]','float32[3]','float32[7]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       arm_name,end_position,end_rpy,angles_current

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MekaIKRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.arm_name is None:
        self.arm_name = ''
      if self.end_position is None:
        self.end_position = [0.,0.,0.]
      if self.end_rpy is None:
        self.end_rpy = [0.,0.,0.]
      if self.angles_current is None:
        self.angles_current = [0.,0.,0.,0.,0.,0.,0.]
    else:
      self.arm_name = ''
      self.end_position = [0.,0.,0.]
      self.end_rpy = [0.,0.,0.]
      self.angles_current = [0.,0.,0.,0.,0.,0.,0.]

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
      _x = self.arm_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_3f.pack(*self.end_position))
      buff.write(_struct_3f.pack(*self.end_rpy))
      buff.write(_struct_7f.pack(*self.angles_current))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.arm_name = str[start:end].decode('utf-8')
      else:
        self.arm_name = str[start:end]
      start = end
      end += 12
      self.end_position = _struct_3f.unpack(str[start:end])
      start = end
      end += 12
      self.end_rpy = _struct_3f.unpack(str[start:end])
      start = end
      end += 28
      self.angles_current = _struct_7f.unpack(str[start:end])
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
      _x = self.arm_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(self.end_position.tostring())
      buff.write(self.end_rpy.tostring())
      buff.write(self.angles_current.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.arm_name = str[start:end].decode('utf-8')
      else:
        self.arm_name = str[start:end]
      start = end
      end += 12
      self.end_position = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.end_rpy = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 28
      self.angles_current = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=7)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7f = struct.Struct("<7f")
_struct_3f = struct.Struct("<3f")
"""autogenerated by genpy from meka_ik/MekaIKResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MekaIKResponse(genpy.Message):
  _md5sum = "f2d3205e4d921a993b3ae1d8ef840cf2"
  _type = "meka_ik/MekaIKResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool success
float32[7] angles_solution

"""
  __slots__ = ['success','angles_solution']
  _slot_types = ['bool','float32[7]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       success,angles_solution

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MekaIKResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.success is None:
        self.success = False
      if self.angles_solution is None:
        self.angles_solution = [0.,0.,0.,0.,0.,0.,0.]
    else:
      self.success = False
      self.angles_solution = [0.,0.,0.,0.,0.,0.,0.]

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
      buff.write(_struct_B.pack(self.success))
      buff.write(_struct_7f.pack(*self.angles_solution))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _struct_B.unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 28
      self.angles_solution = _struct_7f.unpack(str[start:end])
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
      buff.write(_struct_B.pack(self.success))
      buff.write(self.angles_solution.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _struct_B.unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 28
      self.angles_solution = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=7)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7f = struct.Struct("<7f")
_struct_B = struct.Struct("<B")
class MekaIK(object):
  _type          = 'meka_ik/MekaIK'
  _md5sum = 'ad8c64be4db919e70fe6ca282436c423'
  _request_class  = MekaIKRequest
  _response_class = MekaIKResponse
