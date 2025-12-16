# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dh_gripper_msgs:msg/GripperState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GripperState(type):
    """Metaclass of message 'GripperState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dh_gripper_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dh_gripper_msgs.msg.GripperState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gripper_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gripper_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gripper_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gripper_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gripper_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GripperState(metaclass=Metaclass_GripperState):
    """Message class 'GripperState'."""

    __slots__ = [
        '_header',
        '_is_initialized',
        '_grip_state',
        '_position',
        '_target_position',
        '_target_force',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'is_initialized': 'boolean',
        'grip_state': 'int32',
        'position': 'float',
        'target_position': 'float',
        'target_force': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.is_initialized = kwargs.get('is_initialized', bool())
        self.grip_state = kwargs.get('grip_state', int())
        self.position = kwargs.get('position', float())
        self.target_position = kwargs.get('target_position', float())
        self.target_force = kwargs.get('target_force', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.is_initialized != other.is_initialized:
            return False
        if self.grip_state != other.grip_state:
            return False
        if self.position != other.position:
            return False
        if self.target_position != other.target_position:
            return False
        if self.target_force != other.target_force:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def is_initialized(self):
        """Message field 'is_initialized'."""
        return self._is_initialized

    @is_initialized.setter
    def is_initialized(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_initialized' field must be of type 'bool'"
        self._is_initialized = value

    @builtins.property
    def grip_state(self):
        """Message field 'grip_state'."""
        return self._grip_state

    @grip_state.setter
    def grip_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'grip_state' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'grip_state' field must be an integer in [-2147483648, 2147483647]"
        self._grip_state = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position = value

    @builtins.property
    def target_position(self):
        """Message field 'target_position'."""
        return self._target_position

    @target_position.setter
    def target_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_position' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_position' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_position = value

    @builtins.property
    def target_force(self):
        """Message field 'target_force'."""
        return self._target_force

    @target_force.setter
    def target_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_force' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_force' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_force = value
