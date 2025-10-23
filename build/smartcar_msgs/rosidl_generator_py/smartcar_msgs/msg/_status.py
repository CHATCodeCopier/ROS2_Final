# generated from rosidl_generator_py/resource/_idl.py.em
# with input from smartcar_msgs:msg/Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

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
            module = import_type_support('smartcar_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'smartcar_msgs.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Status(metaclass=Metaclass_Status):
    """Message class 'Status'."""

    __slots__ = [
        '_battery_voltage_mv',
        '_battery_current_ma',
        '_battery_percentage',
        '_steering_angle_rad',
        '_engine_speed_rpm',
    ]

    _fields_and_field_types = {
        'battery_voltage_mv': 'int32',
        'battery_current_ma': 'int32',
        'battery_percentage': 'double',
        'steering_angle_rad': 'double',
        'engine_speed_rpm': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.battery_voltage_mv = kwargs.get('battery_voltage_mv', int())
        self.battery_current_ma = kwargs.get('battery_current_ma', int())
        self.battery_percentage = kwargs.get('battery_percentage', float())
        self.steering_angle_rad = kwargs.get('steering_angle_rad', float())
        self.engine_speed_rpm = kwargs.get('engine_speed_rpm', int())

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
        if self.battery_voltage_mv != other.battery_voltage_mv:
            return False
        if self.battery_current_ma != other.battery_current_ma:
            return False
        if self.battery_percentage != other.battery_percentage:
            return False
        if self.steering_angle_rad != other.steering_angle_rad:
            return False
        if self.engine_speed_rpm != other.engine_speed_rpm:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def battery_voltage_mv(self):
        """Message field 'battery_voltage_mv'."""
        return self._battery_voltage_mv

    @battery_voltage_mv.setter
    def battery_voltage_mv(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'battery_voltage_mv' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'battery_voltage_mv' field must be an integer in [-2147483648, 2147483647]"
        self._battery_voltage_mv = value

    @builtins.property
    def battery_current_ma(self):
        """Message field 'battery_current_ma'."""
        return self._battery_current_ma

    @battery_current_ma.setter
    def battery_current_ma(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'battery_current_ma' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'battery_current_ma' field must be an integer in [-2147483648, 2147483647]"
        self._battery_current_ma = value

    @builtins.property
    def battery_percentage(self):
        """Message field 'battery_percentage'."""
        return self._battery_percentage

    @battery_percentage.setter
    def battery_percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_percentage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'battery_percentage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._battery_percentage = value

    @builtins.property
    def steering_angle_rad(self):
        """Message field 'steering_angle_rad'."""
        return self._steering_angle_rad

    @steering_angle_rad.setter
    def steering_angle_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steering_angle_rad' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'steering_angle_rad' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._steering_angle_rad = value

    @builtins.property
    def engine_speed_rpm(self):
        """Message field 'engine_speed_rpm'."""
        return self._engine_speed_rpm

    @engine_speed_rpm.setter
    def engine_speed_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'engine_speed_rpm' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'engine_speed_rpm' field must be an integer in [-2147483648, 2147483647]"
        self._engine_speed_rpm = value
