#ifndef STD_MSGS_PY_WRAPPER_HPP
#define STD_MSGS_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "py_wrapper.hpp"

static builtin_interfaces::msg::Time PyStamp_AsStamp(PyObject* pyStamp) {
    builtin_interfaces::msg::Time cppStamp = builtin_interfaces::msg::Time ();

    cppStamp.sec = PyLong_AsLong(PyObject_GetAttrString(pyStamp, "_sec"));
    cppStamp.nanosec = PyLong_AsUnsignedLong(PyObject_GetAttrString(pyStamp, "_nanosec"));

    return cppStamp;
}

static PyObject* PyStamp_FromStamp(const builtin_interfaces::msg::Time& cppStamp, PyObject* pyStamp = NULL) {
    if (pyStamp == NULL) {
        PyObject* time_class = PyWrapper::GetFunction("builtin_interfaces.msg", "Time");
        pyStamp = PyObject_CallObject(time_class, NULL);
        Py_XDECREF(time_class);
    }
    
    PyObject_SetAttrString(pyStamp, "_sec", PyLong_FromLong(cppStamp.sec));
    PyObject_SetAttrString(pyStamp, "_nanosec", PyLong_FromUnsignedLong(cppStamp.nanosec));
    
    return pyStamp;
}

static std_msgs::msg::Header PyHeader_AsHeader(PyObject* pyHeader) {
    std_msgs::msg::Header cppHeader = std_msgs::msg::Header();

    cppHeader.stamp = PyStamp_AsStamp(PyObject_GetAttrString(pyHeader, "_stamp"));
    cppHeader.frame_id = PyUnicode_AsUTF8(PyObject_GetAttrString(pyHeader, "_frame_id"));

    return cppHeader;
}

static PyObject* PyHeader_FromHeader(const std_msgs::msg::Header& cppHeader, PyObject* pyHeader = NULL) {
    if (pyHeader == NULL) {
        PyObject* header_class = PyWrapper::GetFunction("std_msgs.msg", "Header");
        pyHeader = PyObject_CallObject(header_class, NULL);
        Py_XDECREF(header_class);
    }

    PyObject_SetAttrString(pyHeader, "_stamp", PyStamp_FromStamp(cppHeader.stamp, PyObject_GetAttrString(pyHeader, "_stamp")));
    PyObject_SetAttrString(pyHeader, "_frame_id", PyUnicode_FromString(cppHeader.frame_id.c_str()));

    return pyHeader;
}


#endif // STD_MSGS_PY_WRAPPER_HPP