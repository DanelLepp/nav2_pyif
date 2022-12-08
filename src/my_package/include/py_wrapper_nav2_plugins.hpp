#ifndef PY_WRAPPER_COMPUTE_VELOCITY_HPP
#define PY_WRAPPER_COMPUTE_VELOCITY_HPP

#include "python3.10/Python.h"

#include "py_wrapper_geometry_msgs.hpp"

namespace PyWrapper {
    class Nav2Plugins {
        private:
            PyObject* module_Nav2Plugins = NULL;
            PyObject* computeVelocity = NULL;
            PyObject* arguments = NULL;
            PyObject* value = NULL;
            PyWrapper::GeometryMsgs pyWrapper_GeometryMsgs;

        public:
            Nav2Plugins() {
                if (module_Nav2Plugins == NULL) {
                    module_Nav2Plugins = PyImport_ImportModule("py_package.py_package");
                    if (module_Nav2Plugins == NULL) {
                        std::cout << "module_Nav2Plugins == NULL" << std::endl;
                    }
                }

                if (computeVelocity == NULL) {
                    computeVelocity = PyObject_GetAttrString(module_Nav2Plugins, "ComputeVelocity");
                    if (computeVelocity == NULL) {
                        std::cout << "computeVelocity == NULL" << std::endl;
                    }
                }

                if (arguments == NULL) {
                    arguments = PyTuple_New(2);
                    if (arguments == NULL) {
                        std::cout << "arguments == NULL" << std::endl;
                    }
                }
                
                pyWrapper_GeometryMsgs = PyWrapper::GeometryMsgs();
            }

            ~Nav2Plugins() {
                Py_XDECREF(module_Nav2Plugins);
                Py_XDECREF(computeVelocity);
                Py_XDECREF(arguments);
                
            }

            geometry_msgs::msg::TwistStamped ComputeVelocity(geometry_msgs::msg::PoseStamped& poseStamped, geometry_msgs::msg::Twist& twist) {
                PyTuple_SetItem(arguments, 0, pyWrapper_GeometryMsgs.PyPoseStamped_FromPoseStamped(poseStamped));
                PyTuple_SetItem(arguments, 1, pyWrapper_GeometryMsgs.PyTwist_FromTwist(twist));
                
                value = PyObject_CallObject(computeVelocity, arguments);

                return pyWrapper_GeometryMsgs.PyTwistStamped_AsTwistStamped(value);
            }
    };
}

#endif // PY_WRAPPER_COMPUTE_VELOCITY_HPP