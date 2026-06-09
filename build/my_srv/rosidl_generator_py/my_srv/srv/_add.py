# generated from rosidl_generator_py/resource/_idl.py.em
# with input from my_srv:srv/Add.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Add_Request(type):
    """Metaclass of message 'Add_Request'."""

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
            module = import_type_support('my_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'my_srv.srv.Add_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__add__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__add__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__add__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__add__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__add__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Add_Request(metaclass=Metaclass_Add_Request):
    """Message class 'Add_Request'."""

    __slots__ = [
        '_color',
        '_low_h',
        '_low_s',
        '_low_v',
        '_high_h',
        '_high_s',
        '_high_v',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'color': 'string',
        'low_h': 'int32',
        'low_s': 'int32',
        'low_v': 'int32',
        'high_h': 'int32',
        'high_s': 'int32',
        'high_v': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.color = kwargs.get('color', str())
        self.low_h = kwargs.get('low_h', int())
        self.low_s = kwargs.get('low_s', int())
        self.low_v = kwargs.get('low_v', int())
        self.high_h = kwargs.get('high_h', int())
        self.high_s = kwargs.get('high_s', int())
        self.high_v = kwargs.get('high_v', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.color != other.color:
            return False
        if self.low_h != other.low_h:
            return False
        if self.low_s != other.low_s:
            return False
        if self.low_v != other.low_v:
            return False
        if self.high_h != other.high_h:
            return False
        if self.high_s != other.high_s:
            return False
        if self.high_v != other.high_v:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def color(self):
        """Message field 'color'."""
        return self._color

    @color.setter
    def color(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'color' field must be of type 'str'"
        self._color = value

    @builtins.property
    def low_h(self):
        """Message field 'low_h'."""
        return self._low_h

    @low_h.setter
    def low_h(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'low_h' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'low_h' field must be an integer in [-2147483648, 2147483647]"
        self._low_h = value

    @builtins.property
    def low_s(self):
        """Message field 'low_s'."""
        return self._low_s

    @low_s.setter
    def low_s(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'low_s' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'low_s' field must be an integer in [-2147483648, 2147483647]"
        self._low_s = value

    @builtins.property
    def low_v(self):
        """Message field 'low_v'."""
        return self._low_v

    @low_v.setter
    def low_v(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'low_v' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'low_v' field must be an integer in [-2147483648, 2147483647]"
        self._low_v = value

    @builtins.property
    def high_h(self):
        """Message field 'high_h'."""
        return self._high_h

    @high_h.setter
    def high_h(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'high_h' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'high_h' field must be an integer in [-2147483648, 2147483647]"
        self._high_h = value

    @builtins.property
    def high_s(self):
        """Message field 'high_s'."""
        return self._high_s

    @high_s.setter
    def high_s(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'high_s' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'high_s' field must be an integer in [-2147483648, 2147483647]"
        self._high_s = value

    @builtins.property
    def high_v(self):
        """Message field 'high_v'."""
        return self._high_v

    @high_v.setter
    def high_v(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'high_v' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'high_v' field must be an integer in [-2147483648, 2147483647]"
        self._high_v = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Add_Response(type):
    """Metaclass of message 'Add_Response'."""

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
            module = import_type_support('my_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'my_srv.srv.Add_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__add__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__add__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__add__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__add__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__add__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Add_Response(metaclass=Metaclass_Add_Response):
    """Message class 'Add_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Add_Event(type):
    """Metaclass of message 'Add_Event'."""

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
            module = import_type_support('my_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'my_srv.srv.Add_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__add__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__add__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__add__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__add__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__add__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Add_Event(metaclass=Metaclass_Add_Event):
    """Message class 'Add_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<my_srv/Add_Request, 1>',
        'response': 'sequence<my_srv/Add_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['my_srv', 'srv'], 'Add_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['my_srv', 'srv'], 'Add_Response'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from my_srv.srv import Add_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, Add_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'Add_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from my_srv.srv import Add_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, Add_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'Add_Response'"
        self._response = value


class Metaclass_Add(type):
    """Metaclass of service 'Add'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('my_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'my_srv.srv.Add')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__add

            from my_srv.srv import _add
            if _add.Metaclass_Add_Request._TYPE_SUPPORT is None:
                _add.Metaclass_Add_Request.__import_type_support__()
            if _add.Metaclass_Add_Response._TYPE_SUPPORT is None:
                _add.Metaclass_Add_Response.__import_type_support__()
            if _add.Metaclass_Add_Event._TYPE_SUPPORT is None:
                _add.Metaclass_Add_Event.__import_type_support__()


class Add(metaclass=Metaclass_Add):
    from my_srv.srv._add import Add_Request as Request
    from my_srv.srv._add import Add_Response as Response
    from my_srv.srv._add import Add_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
