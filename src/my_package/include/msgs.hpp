#ifndef MSGS_HPP
#define MSGS_HPP

#include "python3.10/Python.h"


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

class geometry_msgs_wrapper {
    private:
        PyObject* geometry_msgs_module = NULL;
        PyObject* builtin_interfaces_module = NULL;

    public:
        geometry_msgs_wrapper() {
            if (geometry_msgs_module == NULL) {
                geometry_msgs_module = PyImport_ImportModule("geometry_msgs.msg");
                if (geometry_msgs_module == NULL) {
                    std::cout << "pointClass == NULL" << std::endl;
                }
            }

            if (builtin_interfaces_module == NULL) {
                builtin_interfaces_module = PyImport_ImportModule("builtin_interfaces.msg");
                if (builtin_interfaces_module == NULL) {
                    std::cout << "pointClass == NULL" << std::endl;
                }
            }
        }

        ~geometry_msgs_wrapper() {
            Py_XDECREF(geometry_msgs_module);
            Py_XDECREF(builtin_interfaces_module);
        }

        builtin_interfaces::msg::Time From_PyStamp(PyObject* stamp_py) {
            builtin_interfaces::msg::Time stamp_cpp = builtin_interfaces::msg::Time ();

            stamp_cpp.sec = PyLong_AsLong(PyObject_GetAttrString(stamp_py, "_sec"));
            stamp_cpp.nanosec = PyLong_AsUnsignedLong(PyObject_GetAttrString(stamp_py, "_nanosec"));

            return stamp_cpp;
        }

        PyObject* From_CppStamp(builtin_interfaces::msg::Time stamp_cpp) {
            PyObject* time_class = PyObject_GetAttrString(builtin_interfaces_module, "Time");

            PyObject* stamp_py = PyObject_CallObject(time_class, NULL);
            Py_XDECREF(time_class);

            PyObject_SetAttrString(stamp_py, "_sec", PyLong_FromLong(stamp_cpp.sec));
            PyObject_SetAttrString(stamp_py, "_nanosec", PyLong_FromUnsignedLong(stamp_cpp.nanosec));
            
            return stamp_py;
        }

        geometry_msgs::msg::Vector3 From_PyVector3(PyObject* vector3_py) {
            geometry_msgs::msg::Vector3 vector3_cpp = geometry_msgs::msg::Vector3();

            vector3_cpp.x = PyFloat_AsDouble(PyObject_GetAttrString(vector3_py, "_x"));
            vector3_cpp.y = PyFloat_AsDouble(PyObject_GetAttrString(vector3_py, "_y"));
            vector3_cpp.z = PyFloat_AsDouble(PyObject_GetAttrString(vector3_py, "_z"));

            return vector3_cpp;
        }

        PyObject* From_CppVector3(geometry_msgs::msg::Vector3 vector3_cpp) {
            PyObject* vector3_class = NULL;
            vector3_class = PyObject_GetAttrString(geometry_msgs_module, "Vector3");

            PyObject* vector3_py = PyObject_CallObject(vector3_class, NULL);
            Py_XDECREF(vector3_class);

            PyObject_SetAttrString(vector3_py, "_x", PyFloat_FromDouble(vector3_cpp.x));
            PyObject_SetAttrString(vector3_py, "_y", PyFloat_FromDouble(vector3_cpp.y));
            PyObject_SetAttrString(vector3_py, "_z", PyFloat_FromDouble(vector3_cpp.z));

            return vector3_py;
        }

        std_msgs::msg::Header From_PyHeader(PyObject* header_py) {
            std_msgs::msg::Header header_cpp = std_msgs::msg::Header();

            //header_cpp.seq = PyLong_AsUnsignedLong(PyObject_GetAttrString(header_py, "_seq"));
            header_cpp.stamp = From_PyStamp(PyObject_GetAttrString(header_py, "_stamp"));
            header_cpp.frame_id = PyBytes_AsString(PyObject_GetAttrString(header_py, "_frame_id"));

            return header_cpp;
        }

        PyObject* From_CppHeader(std_msgs::msg::Header header_cpp) {
            PyObject* header_class = PyObject_GetAttrString(builtin_interfaces_module, "Time");

            PyObject* header_py = NULL;
            header_py = PyObject_CallObject(header_class, NULL);
            Py_XDECREF(header_class);

            //PyObject_SetAttrString(header_py, "_seq", PyLong_FromLong(header_cpp.seq));
            PyObject_SetAttrString(header_py, "_stamp", From_CppStamp(header_cpp.stamp));
            PyObject_SetAttrString(header_py, "_frame_id", PyBytes_FromString(header_cpp.frame_id.c_str()));
            return header_py;
        }

        geometry_msgs::msg::Point From_PyPoint(PyObject* point_py){
            geometry_msgs::msg::Point point_cpp = geometry_msgs::msg::Point();

            point_cpp.x = PyFloat_AsDouble(PyObject_GetAttrString(point_py, "_x"));
            point_cpp.y = PyFloat_AsDouble(PyObject_GetAttrString(point_py, "_y"));
            point_cpp.z = PyFloat_AsDouble(PyObject_GetAttrString(point_py, "_z"));

            return point_cpp;
        }

        PyObject* From_CppPoint(geometry_msgs::msg::Point point_cpp) {
            PyObject* point_class = NULL;
            point_class = PyObject_GetAttrString(geometry_msgs_module, "Point");

            PyObject* point_py = PyObject_CallObject(point_class, NULL);
            Py_XDECREF(point_class);

            PyObject_SetAttrString(point_py, "_x", PyFloat_FromDouble(point_cpp.x));
            PyObject_SetAttrString(point_py, "_y", PyFloat_FromDouble(point_cpp.y));
            PyObject_SetAttrString(point_py, "_z", PyFloat_FromDouble(point_cpp.z));

            return point_py;
        }

        geometry_msgs::msg::Quaternion From_PyOrientation(PyObject* orientation_py) {
            geometry_msgs::msg::Quaternion orientation_cpp = geometry_msgs::msg::Quaternion();

            orientation_cpp.x = PyFloat_AsDouble(PyObject_GetAttrString(orientation_py, "_x"));
            orientation_cpp.y = PyFloat_AsDouble(PyObject_GetAttrString(orientation_py, "_y"));
            orientation_cpp.z = PyFloat_AsDouble(PyObject_GetAttrString(orientation_py, "_z"));
            orientation_cpp.w = PyFloat_AsDouble(PyObject_GetAttrString(orientation_py, "_w"));

            return orientation_cpp;
        }

        PyObject* From_CppOrientation(geometry_msgs::msg::Quaternion orientation_cpp) {
            PyObject* quaternion_class = NULL;
            quaternion_class = PyObject_GetAttrString(geometry_msgs_module, "Quaternion");

            PyObject* orientation_py = PyObject_CallObject(quaternion_class, NULL);
            Py_XDECREF(quaternion_class);

            PyObject_SetAttrString(orientation_py, "_x", PyFloat_FromDouble(orientation_cpp.x));
            PyObject_SetAttrString(orientation_py, "_y", PyFloat_FromDouble(orientation_cpp.y));
            PyObject_SetAttrString(orientation_py, "_z", PyFloat_FromDouble(orientation_cpp.z));
            PyObject_SetAttrString(orientation_py, "_w", PyFloat_FromDouble(orientation_cpp.w));

            return orientation_py;
        }

        geometry_msgs::msg::Pose From_PyPose(PyObject* pose_py) {
            geometry_msgs::msg::Pose pose_cpp = geometry_msgs::msg::Pose();

            pose_cpp.position = From_PyPoint(PyObject_GetAttrString(pose_py, "_position"));
            pose_cpp.orientation = From_PyOrientation(PyObject_GetAttrString(pose_py, "_orientation"));

            return pose_cpp;
        }

        PyObject* From_CppPose(geometry_msgs::msg::Pose pose_cpp) {
            PyObject* pose_class = NULL;
            pose_class = PyObject_GetAttrString(geometry_msgs_module, "Pose");

            PyObject* pose_py = PyObject_CallObject(pose_class, NULL);
            Py_XDECREF(pose_class);

            PyObject_SetAttrString(pose_py, "_position", From_CppPoint(pose_cpp.position));
            PyObject_SetAttrString(pose_py, "_orientation", From_CppOrientation(pose_cpp.orientation));

            return pose_py;
        }


        geometry_msgs::msg::PoseStamped From_PyPoseStamped(PyObject* pose_stamped_py){
            geometry_msgs::msg::PoseStamped pose_stamped_cpp = geometry_msgs::msg::PoseStamped();

            pose_stamped_cpp.header = From_PyHeader(PyObject_GetAttrString(pose_stamped_py, "_header"));
            pose_stamped_cpp.pose = From_PyPose(PyObject_GetAttrString(pose_stamped_py, "_pose"));


            return pose_stamped_cpp;
        }

        PyObject* From_CppPoseStamped(geometry_msgs::msg::PoseStamped pose_stamped_cpp) {
            PyObject* pose_stamped_class = NULL;
            pose_stamped_class = PyObject_GetAttrString(geometry_msgs_module, "PoseStamped");

            PyObject* pose_stamped_py = PyObject_CallObject(pose_stamped_class, NULL);
            Py_XDECREF(pose_stamped_class);

            PyObject_SetAttrString(pose_stamped_py, "_header", From_CppHeader(pose_stamped_cpp.header));
            PyObject_SetAttrString(pose_stamped_py, "_pose", From_CppPose(pose_stamped_cpp.pose));

            return pose_stamped_py;
        }

        geometry_msgs::msg::Twist From_PyTwist(PyObject* twist_py){
            geometry_msgs::msg::Twist twist_cpp = geometry_msgs::msg::Twist();

            twist_cpp.linear = From_PyVector3(PyObject_GetAttrString(twist_py, "_linear"));
            twist_cpp.angular = From_PyVector3(PyObject_GetAttrString(twist_py, "_angular"));

            return twist_cpp;
        }

        PyObject* From_CppTwist(geometry_msgs::msg::Twist twist_cpp) {
            PyObject* twist_class = NULL;
            twist_class = PyObject_GetAttrString(geometry_msgs_module, "Twist");

            PyObject* twist_py = PyObject_CallObject(twist_class, NULL);
            Py_XDECREF(twist_class);

            PyObject_SetAttrString(twist_py, "_linear", From_CppVector3(twist_cpp.linear));
            PyObject_SetAttrString(twist_py, "_angular", From_CppVector3(twist_cpp.angular));

            return twist_py;
        }

        geometry_msgs::msg::TwistStamped From_PyTwistStamped(PyObject* twist_stamped_py){
            geometry_msgs::msg::TwistStamped twist_stamped_cpp = geometry_msgs::msg::TwistStamped();

            twist_stamped_cpp.header = From_PyHeader(PyObject_GetAttrString(twist_stamped_py, "_header"));
            twist_stamped_cpp.twist = From_PyTwist(PyObject_GetAttrString(twist_stamped_py, "_twist"));

            return twist_stamped_cpp;
        }

        PyObject* From_CppTwistStamped(geometry_msgs::msg::TwistStamped twist_stamped_cpp) {
            PyObject* twist_stamped_class = NULL;
            twist_stamped_class = PyObject_GetAttrString(geometry_msgs_module, "TwistStamped");

            PyObject* twist_stamped_py = PyObject_CallObject(twist_stamped_class, NULL);
            Py_XDECREF(twist_stamped_class);

            PyObject_SetAttrString(twist_stamped_py, "_header", From_CppHeader(twist_stamped_cpp.header));
            PyObject_SetAttrString(twist_stamped_py, "_twist", From_CppTwist(twist_stamped_cpp.twist));

            return twist_stamped_py;
        }
};

#endif // MSGS_HPP