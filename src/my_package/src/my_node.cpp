#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "python3.10/Python.h"

PyObject* AttachDataToPyMessage(std_msgs::msg::String ros_message) {
  // passing data to Python std_msg struct
  PyObject* pymessage_module = PyImport_ImportModule("std_msgs.msg._string");
  PyObject* pymessage_class = PyObject_GetAttrString(pymessage_module, "String");
  PyObject* pymessage = PyObject_CallObject(pymessage_class, NULL);

  std::string data = ros_message.data;
  PyObject* field = PyUnicode_DecodeUTF8(data.c_str(), data.length(), "strict");

  PyObject_SetAttrString(pymessage, "data", field);

  return pymessage;
}

PyObject* RunEmbeddedMethodWithArgs(PyObject* pymessage) {
  PyObject* pName = PyUnicode_FromString("py_package.py_package");
  PyObject* pModule = PyImport_Import(pName);
  PyObject* pFunc = PyObject_GetAttrString(pModule, "Concatenate");
  PyObject* pArgs = PyTuple_New(1);

  PyTuple_SetItem(pArgs, 0, pymessage);

  PyObject* pValue = PyObject_CallObject(pFunc, pArgs);

  return pValue;
}

std::string ConvertPyMessageToCPPmessage(PyObject* pValue) {
  PyObject* field = PyObject_GetAttrString(pValue, "data");
  PyObject* encoded_field = PyUnicode_AsUTF8String(field);
  std::string ret = PyBytes_AS_STRING(encoded_field);
  
  return ret;
}

int main(int argc, char ** argv) {
  Py_Initialize();

  auto message = std_msgs::msg::String();
  message.data = "Hello ";
  
  PyObject* pymessage = AttachDataToPyMessage(message);
  PyObject* pValue = RunEmbeddedMethodWithArgs(pymessage);
  message.data = ConvertPyMessageToCPPmessage(pValue);

  std::cout << message.data << std::endl;
  Py_Finalize();
  return 0;
}
// ros2 run my_package my_node
