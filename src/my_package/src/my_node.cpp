#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


/* From C-Pyhton API:

Note

For all # variants of formats (s#, y#, etc.), 
the macro PY_SSIZE_T_CLEAN must be defined before including Python.h. 
On Python 3.9 and older, the type of the length argument is Py_ssize_t 
if the PY_SSIZE_T_CLEAN macro is defined, or int otherwise. 

https://docs.python.org/3/c-api/arg.html#arg-parsing
*/
#define PY_SSIZE_T_CLEAN
#include "python3.10/Python.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "msgs.hpp"

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

void importingNumpy() {
  PyRun_SimpleString("import numpy");
}

PyObject* setPyTuple() 
{
  /* Objects all initialized to NULL for Py_XDECREF */
  PyObject *geometry_msgs_module1 = NULL, *pose_stamped_instance = NULL, *pose_instance = NULL, *point_instance = NULL,
    *pose_stamped_class = NULL, *pose_class = NULL, *point_class = NULL;

  geometry_msgs_module1 = PyImport_ImportModule("geometry_msgs.msg"); // _pose_stamped.py was the actual file
  if (geometry_msgs_module1 == NULL) {
    std::cout << "geometry_msgs_module == NULL" << std::endl;
  }

  //geometry_msgs_module();
  geometry_msgs::msg::Point point_cpp = geometry_msgs::msg::Point();
  point_cpp.x = 10.0;
  point_cpp.y = 2.0;
  point_cpp.z = 3.0;

  geometry_msgs_wrapper GeoMetryMsgWrapper = geometry_msgs_wrapper();;
  PyObject* point_py = GeoMetryMsgWrapper.From_CppPoint(point_cpp);

  if (point_py == NULL) {
    std::cout << "pointClass == NULL" << std::endl;
    return NULL;
  }

  // std_msgs::msg::Header header_cpp = std_msgs::msg::Header();
  // header_cpp.seq = 30;
  // point_class = PyObject_GetAttrString(geometry_msgs_module1, "Point");  //class
  // if (point_class == NULL) {
  //   std::cout << "pointClass == NULL" << std::endl;
  //   return NULL;
  // }

  // PyObject *pointKeyWords = PyDict_New();
  // PyDict_SetItemString(pointKeyWords, "x", PyFloat_FromDouble(1.0));
  // PyDict_SetItemString(pointKeyWords, "y", PyFloat_FromDouble(2.0));
  // PyDict_SetItemString(pointKeyWords, "z", PyFloat_FromDouble(3.0));
  // // PyObject *pointArgs= PyTuple_New(3);
  // // PyTuple_SetItem(pointArgs, 0, Py_BuildValue("X"));
  // // PyTuple_SetItem(pointArgs, 1, Py_BuildValue("Y"));
  // // PyTuple_SetItem(pointArgs, 2, Py_BuildValue("Z"));
  // point_instance = PyObject_CallObject(point_class, NULL);  //class
  // //point_instance = PyObject_VectorcallDict(point_class, 3, );
  // //point_instance = PyObject_CallObject(point_class, PyTuple_Pack(1.0, 2.0, 3.0));
  // if (point_instance == NULL) {
  //   std::cout << "point_instance == NULL" << std::endl;
  //   return NULL;
  // }
  // else {
  //   std::cout << "point_init" << std::endl;
  //   PyObject_SetAttrString(point_instance, "_x", PyFloat_FromDouble(1.0));
  //   PyObject_SetAttrString(point_instance, "_y", PyFloat_FromDouble(2.0));
  //   PyObject_SetAttrString(point_instance, "_z", PyFloat_FromDouble(3.0));
  //   //PyObject_CallMethodObjArgs(point_instance, PyObject_GetAttrString(point_instance, "__init__"), pointKeyWords, NULL);
  // }

  // pose_class = PyObject_GetAttrString(geometry_msgs_module1, "Pose");  //class
  // if (pose_class == NULL) {
  //   std::cout << "poseClass == NULL" << std::endl;
  //   return NULL;
  // }
  // pose_instance = PyObject_CallObject(pose_class, point.Ptr());


  // pose_stamped_class = PyObject_GetAttrString(geometry_msgs_module1, "PoseStamped");  //class
  // if (pose_stamped_class == NULL) {
  //   std::cout << "poseStampedClass == NULL" << std::endl;
  //   return NULL;
  // }
  // pose_stamped_instance = PyObject_CallObject(pose_stamped_class, pose_instance);


  PyObject *pName = NULL, *pModule = NULL, *pFunc = NULL, *pArgs = NULL, *pValue = NULL;

  pName = PyUnicode_FromString("py_package.py_package");
  if (pName == NULL) {
    std::cout << "pName == NULL" << std::endl;
    return NULL;
  }

  pModule = PyImport_Import(pName);
  Py_XDECREF(pName);
  if (pModule == NULL) {
    std::cout << "pModule == NULL" << std::endl;
    return NULL;
  }

  pFunc = PyObject_GetAttrString(pModule, "ComputeVelocity");
  Py_XDECREF(pModule);
  if (pFunc == NULL) {
    std::cout << "pFunc == NULL" << std::endl;
    return NULL;
  }

  pArgs = PyTuple_New(1);
  if (pArgs == NULL) {
    std::cout << "pArgs == NULL" << std::endl;
    return NULL;
  }

  int tuple = PyTuple_SetItem(pArgs, 0, point_py);
  if (tuple < 0) {
    std::cout << "set tuple failed" << std::endl;
    return NULL;
  }

  pValue = PyObject_CallObject(pFunc, pArgs);
  Py_XDECREF(pArgs);
  if (pValue == NULL) {
    std::cout << "pValue == NULL" << std::endl;
    return NULL;
  }

  return NULL;
}

PyObject* callComputeVelocity(PyObject* poseStamped) {
  /* Objects all initialized to NULL for Py_XDECREF */
  PyObject *pName = NULL, *pModule = NULL, *pFunc = NULL, *pArgs = NULL, *pValue = NULL;

  pName = PyUnicode_FromString("py_package.py_package");
  if (pName == NULL) {
    std::cout << "pName == NULL" << std::endl;
    return NULL;
  }

  pModule = PyImport_Import(pName);
  Py_XDECREF(pName);
  if (pModule == NULL) {
    std::cout << "pModule == NULL" << std::endl;
    return NULL;
  }

  pFunc = PyObject_GetAttrString(pModule, "ComputeVelocity");
  Py_XDECREF(pModule);
  if (pFunc == NULL) {
    std::cout << "pFunc == NULL" << std::endl;
    return NULL;
  }

  pArgs = PyTuple_New(1);
  if (pArgs == NULL) {
    std::cout << "pArgs == NULL" << std::endl;
    return NULL;
  }

  int tuple = PyTuple_SetItem(pArgs, 0, poseStamped);
  if (tuple < 0) {
    std::cout << "set tuple failed" << std::endl;
    return NULL;
  }

  pValue = PyObject_CallObject(pFunc, pArgs);
  Py_XDECREF(pArgs);
  if (pValue == NULL) {
    std::cout << "pValue == NULL" << std::endl;
    return NULL;
  }

  return pValue;
}

int main() {
  Py_Initialize();
  PyRun_SimpleString("import os");
  PyRun_SimpleString("print(os.path.abspath(os.getcwd()))" );
  PyRun_SimpleString("from py_package import py_package" );
  //importingNumpy();
  auto message = std_msgs::msg::String();
  message.data = "Hello ";
  
  // PyObject* pymessage = AttachDataToPyMessage(message);
  // PyObject* pValue = RunEmbeddedMethodWithArgs(pymessage);
  // message.data = ConvertPyMessageToCPPmessage(pValue);
  // std::cout << message.data << std::endl;

  PyObject* pyPoseStamped = setPyTuple();
  if (pyPoseStamped == NULL) {
    std::cout << "pyPoseStamped == NULL" << std::endl;
  }
  else {
    //callComputeVelocity(pyPoseStamped);
  }
  
  Py_Finalize();
  return 0;
}
// ros2 run my_package my_node
