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

std_msgs::msg::Header StdMsgs::PyHeader_AsHeader(PyObject* py_header) {
    std_msgs::msg::Header cpp_header = std_msgs::msg::Header();

    cpp_header.stamp = PyStamp_AsStamp(PyObject_GetAttrString(py_header, "_stamp"));
    cpp_header.frame_id = PyUnicode_AsUTF8(PyObject_GetAttrString(py_header, "_frame_id"));

    return cpp_header;
}

PyObject* StdMsgs::PyHeader_FromHeader(const std_msgs::msg::Header& cpp_header, PyObject* py_header) {
    if (py_header == NULL) {
        PyObject* header_class = pyif::PyMap::GetFunction("std_msgs.msg", "Header");
        py_header = PyObject_CallObject(header_class, NULL);
        Py_XDECREF(header_class);
    }

    PyObject_SetAttrString(py_header, "_stamp", PyStamp_FromStamp(cpp_header.stamp, PyObject_GetAttrString(py_header, "_stamp")));
    PyObject_SetAttrString(py_header, "_frame_id", PyUnicode_FromString(cpp_header.frame_id.c_str()));

    return py_header;
}

} // namespace pyif

#endif // PYIF_STD_MSGS_HPP