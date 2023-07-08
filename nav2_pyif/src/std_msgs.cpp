#include "nav2_pyif/std_msgs.hpp"

builtin_interfaces::msg::Time pyif::StdMsgs::PyStamp_AsStamp(PyObject* py_stamp) {
    builtin_interfaces::msg::Time cpp_stamp = builtin_interfaces::msg::Time ();

    cpp_stamp.sec = PyLong_AsLong(PyObject_GetAttrString(py_stamp, "_sec"));
    cpp_stamp.nanosec = PyLong_AsUnsignedLong(PyObject_GetAttrString(py_stamp, "_nanosec"));

    return cpp_stamp;
}

PyObject* pyif::StdMsgs::PyStamp_FromStamp(const builtin_interfaces::msg::Time& cpp_stamp, PyObject* py_stamp) {
    if (py_stamp == NULL) {
        PyObject* time_class = PyMap::GetFunction("builtin_interfaces.msg", "Time");
        py_stamp = PyObject_CallObject(time_class, NULL);
        Py_XDECREF(time_class);
    }
    
    PyObject_SetAttrString(py_stamp, "_sec", PyLong_FromLong(cpp_stamp.sec));
    PyObject_SetAttrString(py_stamp, "_nanosec", PyLong_FromUnsignedLong(cpp_stamp.nanosec));
    
    return py_stamp;
}

std_msgs::msg::Header pyif::StdMsgs::PyHeader_AsHeader(PyObject* py_header) {
    std_msgs::msg::Header cpp_header = std_msgs::msg::Header();

    cpp_header.stamp = PyStamp_AsStamp(PyObject_GetAttrString(py_header, "_stamp"));
    cpp_header.frame_id = PyUnicode_AsUTF8(PyObject_GetAttrString(py_header, "_frame_id"));

    return cpp_header;
}

PyObject* pyif::StdMsgs::PyHeader_FromHeader(const std_msgs::msg::Header& cpp_header, PyObject* py_header) {
    if (py_header == NULL) {
        PyObject* header_class = pyif::PyMap::GetFunction("std_msgs.msg", "Header");
        py_header = PyObject_CallObject(header_class, NULL);
        Py_XDECREF(header_class);
    }

    PyObject_SetAttrString(py_header, "_stamp", PyStamp_FromStamp(cpp_header.stamp, PyObject_GetAttrString(py_header, "_stamp")));
    PyObject_SetAttrString(py_header, "_frame_id", PyUnicode_FromString(cpp_header.frame_id.c_str()));

    return py_header;
}