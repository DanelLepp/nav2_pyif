#ifndef APF_PY_WRAPPER_HPP
#define APF_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN
#include "geometry_msgs_py_wrapper.hpp"

class ArtificialPotentialField {
    private:
        PyObject* module_Apf = NULL;
        PyObject* func_getVelocity = NULL;
        PyObject* arguments = NULL;
        PyObject* value = NULL;
        GeometryMsgs pyWrapper_GeometryMsgs;

    public:
        ArtificialPotentialField() {
            if (module_Apf == NULL) {
                module_Apf = PyImport_ImportModule("artificial_potential_field.artificial_potential_field");
                if (module_Apf == NULL) {
                    std::cout << "module_Apf == NULL" << std::endl;
                }
            }

            if (func_getVelocity == NULL) {
                func_getVelocity = PyObject_GetAttrString(module_Apf, "getVelocity");
                if (func_getVelocity == NULL) {
                    std::cout << "getVelocity == NULL" << std::endl;
                }
            }

            if (arguments == NULL) {
                arguments = PyTuple_New(4);
                if (arguments == NULL) {
                    std::cout << "arguments == NULL" << std::endl;
                }
            }
            
            pyWrapper_GeometryMsgs = GeometryMsgs();
        }

        ~ArtificialPotentialField() {
            Py_XDECREF(module_Apf);
            Py_XDECREF(func_getVelocity);
            Py_XDECREF(arguments);
            
        }
        geometry_msgs::msg::TwistStamped getVelocity(
            sensor_msgs::msg::LaserScan& laserScan, 
            nav_msgs::msg::Odometry& odom, 
            geometry_msgs::msg::PoseStamped& goalPose, 
            nav_msgs::msg::Path& globalPath) 
        {
            // PyTuple_SetItem(arguments, 0, pyWrapper_GeometryMsgs.PyPoseStamped_FromPoseStamped(laserScan));
            // PyTuple_SetItem(arguments, 1, pyWrapper_GeometryMsgs.PyTwist_FromTwist(odom));
            // PyTuple_SetItem(arguments, 2, pyWrapper_GeometryMsgs.PyPoseStamped_FromPoseStamped(goalPose));
            // PyTuple_SetItem(arguments, 3, pyWrapper_GeometryMsgs.PyTwist_FromTwist(globalPath));

            value = PyObject_CallObject(func_getVelocity, arguments);

            return pyWrapper_GeometryMsgs.PyTwistStamped_AsTwistStamped(value);
        }
};


#endif // APF_PY_WRAPPER_HPP