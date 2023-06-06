#ifndef PYIF_STD_MSGS_HPP
#define PYIF_STD_MSGS_HPP

#include "python_interface.hpp"

namespace pyif {
    
builtin_interfaces::msg::Time StdMsgs::PyStamp_AsStamp(PyObject* pyStamp) {
    builtin_interfaces::msg::Time cppStamp = builtin_interfaces::msg::Time ();

    cppStamp.sec = PyLong_AsLong(PyObject_GetAttrString(pyStamp, "_sec"));
    cppStamp.nanosec = PyLong_AsUnsignedLong(PyObject_GetAttrString(pyStamp, "_nanosec"));

    return cppStamp;
}

PyObject* StdMsgs::PyStamp_FromStamp(const builtin_interfaces::msg::Time& cppStamp, PyObject* pyStamp) {
    if (pyStamp == NULL) {
        PyObject* time_class = PyMap::GetFunction("builtin_interfaces.msg", "Time");
        pyStamp = PyObject_CallObject(time_class, NULL);
        Py_XDECREF(time_class);
    }
    
    PyObject_SetAttrString(pyStamp, "_sec", PyLong_FromLong(cppStamp.sec));
    PyObject_SetAttrString(pyStamp, "_nanosec", PyLong_FromUnsignedLong(cppStamp.nanosec));
    
    return pyStamp;
}

std_msgs::msg::Header StdMsgs::PyHeader_AsHeader(PyObject* pyHeader) {
    std_msgs::msg::Header cppHeader = std_msgs::msg::Header();

    cppHeader.stamp = PyStamp_AsStamp(PyObject_GetAttrString(pyHeader, "_stamp"));
    cppHeader.frame_id = PyUnicode_AsUTF8(PyObject_GetAttrString(pyHeader, "_frame_id"));

    return cppHeader;
}

PyObject* StdMsgs::PyHeader_FromHeader(const std_msgs::msg::Header& cppHeader, PyObject* pyHeader) {
    if (pyHeader == NULL) {
        PyObject* header_class = pyif::PyMap::GetFunction("std_msgs.msg", "Header");
        pyHeader = PyObject_CallObject(header_class, NULL);
        Py_XDECREF(header_class);
    }

    PyObject_SetAttrString(pyHeader, "_stamp", PyStamp_FromStamp(cppHeader.stamp, PyObject_GetAttrString(pyHeader, "_stamp")));
    PyObject_SetAttrString(pyHeader, "_frame_id", PyUnicode_FromString(cppHeader.frame_id.c_str()));

    return pyHeader;
}

}; // namespace pyif

#endif // PYIF_STD_MSGS_HPP