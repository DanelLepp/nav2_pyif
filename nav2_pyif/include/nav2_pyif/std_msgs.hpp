#ifndef PYIF_STD_MSGS_HPP
#define PYIF_STD_MSGS_HPP

#include "std_msgs/msg/header.hpp"
#include "python_interface.hpp"

namespace pyif {
    
class StdMsgs {
    public:
        static builtin_interfaces::msg::Time PyStamp_AsStamp(PyObject* py_stamp);

        static PyObject* PyStamp_FromStamp(const builtin_interfaces::msg::Time& cpp_stamp, PyObject* py_stamp);

        static std_msgs::msg::Header PyHeader_AsHeader(PyObject* py_header);

        static PyObject* PyHeader_FromHeader(const std_msgs::msg::Header& cpp_header, PyObject* py_header);
};

} // namespace pyif

#endif // PYIF_STD_MSGS_HPP