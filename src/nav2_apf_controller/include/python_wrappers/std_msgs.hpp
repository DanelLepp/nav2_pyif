#ifndef STD_MSGS_PY_WRAPPER_HPP
#define STD_MSGS_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"


class StdMsgs {
    private:
        PyObject* module_BuiltinInterfaces = NULL;
        PyObject* module_StandardMsgs = NULL;

        static StdMsgs* pinstance_;
        static std::mutex mutex_;

    protected:
        StdMsgs() {
            if (module_BuiltinInterfaces == NULL) {
                module_BuiltinInterfaces = PyImport_ImportModule("builtin_interfaces.msg");
                if (module_BuiltinInterfaces == NULL) {
                    std::cout << "module_BuiltinInterfaces == NULL" << std::endl;
                }
            }

            if (module_StandardMsgs == NULL) {
                module_StandardMsgs = PyImport_ImportModule("std_msgs.msg");
                if (module_StandardMsgs == NULL) {
                    std::cout << "module_StandardMsgs == NULL" << std::endl;
                }
            }
        }

        ~StdMsgs() {
            Py_XDECREF(module_BuiltinInterfaces);
            Py_XDECREF(module_StandardMsgs);
        }

    public:
        StdMsgs(StdMsgs& other) = delete;
        void operator=(const StdMsgs&) = delete;

        static StdMsgs* GetInstance();

        builtin_interfaces::msg::Time PyStamp_AsStamp(PyObject* pyStamp) {
            builtin_interfaces::msg::Time cppStamp = builtin_interfaces::msg::Time ();

            cppStamp.sec = PyLong_AsLong(PyObject_GetAttrString(pyStamp, "_sec"));
            cppStamp.nanosec = PyLong_AsUnsignedLong(PyObject_GetAttrString(pyStamp, "_nanosec"));

            return cppStamp;
        }

        PyObject* PyStamp_FromStamp(const builtin_interfaces::msg::Time& cppStamp, PyObject* pyStamp = NULL) {
            if (pyStamp == NULL) {
                PyObject* time_class = PyObject_GetAttrString(module_BuiltinInterfaces, "Time");

                pyStamp = PyObject_CallObject(time_class, NULL);
                Py_XDECREF(time_class);
            }
            
            PyObject_SetAttrString(pyStamp, "_sec", PyLong_FromLong(cppStamp.sec));
            PyObject_SetAttrString(pyStamp, "_nanosec", PyLong_FromUnsignedLong(cppStamp.nanosec));
            
            return pyStamp;
        }

        std_msgs::msg::Header PyHeader_AsHeader(PyObject* pyHeader) {
            std_msgs::msg::Header cppHeader = std_msgs::msg::Header();

            cppHeader.stamp = PyStamp_AsStamp(PyObject_GetAttrString(pyHeader, "_stamp"));
            cppHeader.frame_id = PyUnicode_AsUTF8(PyObject_GetAttrString(pyHeader, "_frame_id"));

            return cppHeader;
        }

        PyObject* PyHeader_FromHeader(const std_msgs::msg::Header& cppHeader, PyObject* pyHeader = NULL) {
            if (pyHeader == NULL) {
                PyObject* header_class = PyObject_GetAttrString(module_StandardMsgs, "Header");

                pyHeader = PyObject_CallObject(header_class, NULL);
                Py_XDECREF(header_class);
            }

            PyObject_SetAttrString(pyHeader, "_stamp", PyStamp_FromStamp(cppHeader.stamp, PyObject_GetAttrString(pyHeader, "_stamp")));
            PyObject_SetAttrString(pyHeader, "_frame_id", PyUnicode_FromString(cppHeader.frame_id.c_str()));

            return pyHeader;
        }
};

StdMsgs* StdMsgs::pinstance_{nullptr};
std::mutex StdMsgs::mutex_;

StdMsgs* StdMsgs::GetInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr) {
        pinstance_ = new StdMsgs();
    }
    return pinstance_;
}

#endif // STD_MSGS_PY_WRAPPER_HPP