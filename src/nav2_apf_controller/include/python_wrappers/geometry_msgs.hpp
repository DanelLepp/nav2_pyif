#ifndef GEOMETRY_MSGS_PY_WRAPPER_HPP
#define GEOMETRY_MSGS_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#include "std_msgs.hpp"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class GeometryMsgs {
    private:
        PyObject* module_GeometryMsgs = NULL;
        StdMsgs* stdMsgs;

        static GeometryMsgs* pinstance_;
        static std::mutex mutex_;
        
    protected:
        GeometryMsgs() {
            if (module_GeometryMsgs == NULL) {
                module_GeometryMsgs = PyImport_ImportModule("geometry_msgs.msg");
                if (module_GeometryMsgs == NULL) {
                    std::cout << "module_GeometryMsgs == NULL" << std::endl;
                }
            }

            stdMsgs = StdMsgs::GetInstance();
        }

        ~GeometryMsgs() {
            Py_XDECREF(module_GeometryMsgs); 
        }

    public:
        GeometryMsgs(GeometryMsgs& other) = delete;
        void operator=(const GeometryMsgs&) = delete;

        static GeometryMsgs* GetInstance();


        geometry_msgs::msg::Vector3 PyVector3_AsVector3(PyObject* pyVector3) {
            geometry_msgs::msg::Vector3 cppVector3 = geometry_msgs::msg::Vector3();

            cppVector3.x = PyFloat_AsDouble(PyObject_GetAttrString(pyVector3, "_x"));
            cppVector3.y = PyFloat_AsDouble(PyObject_GetAttrString(pyVector3, "_y"));
            cppVector3.z = PyFloat_AsDouble(PyObject_GetAttrString(pyVector3, "_z"));

            return cppVector3;
        }

        PyObject* PyVector3_FromVector3(const geometry_msgs::msg::Vector3& cppVector3, PyObject* pyVector3 = NULL) {
            if (pyVector3 == NULL) {
                PyObject* vector3_class = NULL;
                vector3_class = PyObject_GetAttrString(module_GeometryMsgs, "Vector3");

                pyVector3 = PyObject_CallObject(vector3_class, NULL);
                Py_XDECREF(vector3_class);
            }
            
            PyObject_SetAttrString(pyVector3, "_x", PyFloat_FromDouble(cppVector3.x));
            PyObject_SetAttrString(pyVector3, "_y", PyFloat_FromDouble(cppVector3.y));
            PyObject_SetAttrString(pyVector3, "_z", PyFloat_FromDouble(cppVector3.z));

            return pyVector3;
        }

        geometry_msgs::msg::Point PyPoint_AsPoint(PyObject* pyPoint){
            geometry_msgs::msg::Point cppPoint = geometry_msgs::msg::Point();

            cppPoint.x = PyFloat_AsDouble(PyObject_GetAttrString(pyPoint, "_x"));
            cppPoint.y = PyFloat_AsDouble(PyObject_GetAttrString(pyPoint, "_y"));
            cppPoint.z = PyFloat_AsDouble(PyObject_GetAttrString(pyPoint, "_z"));

            return cppPoint;
        }

        PyObject* PyPoint_FromPoint(const geometry_msgs::msg::Point& cppPoint, PyObject* pyPoint = NULL) {
            if (pyPoint == NULL) {
                PyObject* point_class = NULL;
                point_class = PyObject_GetAttrString(module_GeometryMsgs, "Point");

                pyPoint = PyObject_CallObject(point_class, NULL);
                Py_XDECREF(point_class);
            }
            
            PyObject_SetAttrString(pyPoint, "_x", PyFloat_FromDouble(cppPoint.x));
            PyObject_SetAttrString(pyPoint, "_y", PyFloat_FromDouble(cppPoint.y));
            PyObject_SetAttrString(pyPoint, "_z", PyFloat_FromDouble(cppPoint.z));

            return pyPoint;
        }

        geometry_msgs::msg::Quaternion PyOrientation_AsOrientation(PyObject* pyOrientation) {
            geometry_msgs::msg::Quaternion cppOrientation = geometry_msgs::msg::Quaternion();

            cppOrientation.x = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_x"));
            cppOrientation.y = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_y"));
            cppOrientation.z = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_z"));
            cppOrientation.w = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_w"));

            return cppOrientation;
        }

        PyObject* PyOrientation_FromOrientation(const geometry_msgs::msg::Quaternion& cppOrientation, PyObject* pyOrientation = NULL) {
            if (pyOrientation == NULL) {
                PyObject* quaternion_class = NULL;
                quaternion_class = PyObject_GetAttrString(module_GeometryMsgs, "Quaternion");

                pyOrientation = PyObject_CallObject(quaternion_class, NULL);
                Py_XDECREF(quaternion_class);
            }
            
            PyObject_SetAttrString(pyOrientation, "_x", PyFloat_FromDouble(cppOrientation.x));
            PyObject_SetAttrString(pyOrientation, "_y", PyFloat_FromDouble(cppOrientation.y));
            PyObject_SetAttrString(pyOrientation, "_z", PyFloat_FromDouble(cppOrientation.z));
            PyObject_SetAttrString(pyOrientation, "_w", PyFloat_FromDouble(cppOrientation.w));

            return pyOrientation;
        }

        geometry_msgs::msg::Pose PyPose_AsPose(PyObject* pyPose) {
            geometry_msgs::msg::Pose cppPose = geometry_msgs::msg::Pose();

            cppPose.position = PyPoint_AsPoint(PyObject_GetAttrString(pyPose, "_position"));
            cppPose.orientation = PyOrientation_AsOrientation(PyObject_GetAttrString(pyPose, "_orientation"));

            return cppPose;
        }

        PyObject* PyPose_FromPose(const geometry_msgs::msg::Pose& cppPose, PyObject* pyPose = NULL) {
            if (pyPose == NULL) {
                PyObject* pose_class = NULL;
                pose_class = PyObject_GetAttrString(module_GeometryMsgs, "Pose");

                pyPose = PyObject_CallObject(pose_class, NULL);
                Py_XDECREF(pose_class);
            }
            
            PyObject_SetAttrString(pyPose, "_position", PyPoint_FromPoint(cppPose.position, PyObject_GetAttrString(pyPose, "_position")));
            PyObject_SetAttrString(pyPose, "_orientation", PyOrientation_FromOrientation(cppPose.orientation, PyObject_GetAttrString(pyPose, "_orientation")));

            return pyPose;
        }


        geometry_msgs::msg::PoseStamped PyPoseStamped_AsPoseStamped(PyObject* pyPoseStamped){
            geometry_msgs::msg::PoseStamped cppPoseStamped = geometry_msgs::msg::PoseStamped();

            cppPoseStamped.header = stdMsgs->PyHeader_AsHeader(PyObject_GetAttrString(pyPoseStamped, "_header"));
            cppPoseStamped.pose = PyPose_AsPose(PyObject_GetAttrString(pyPoseStamped, "_pose"));


            return cppPoseStamped;
        }

        PyObject* PyPoseStamped_FromPoseStamped(const geometry_msgs::msg::PoseStamped& cppPoseStamped, PyObject* pyPoseStamped = NULL) {
            if (pyPoseStamped == NULL) {
                PyObject* pose_stamped_class = NULL;
                pose_stamped_class = PyObject_GetAttrString(module_GeometryMsgs, "PoseStamped");

                pyPoseStamped = PyObject_CallObject(pose_stamped_class, NULL);
                Py_XDECREF(pose_stamped_class);
            }
            
            PyObject_SetAttrString(pyPoseStamped, "_header", stdMsgs->PyHeader_FromHeader(cppPoseStamped.header, PyObject_GetAttrString(pyPoseStamped, "_header")));
            PyObject_SetAttrString(pyPoseStamped, "_pose", PyPose_FromPose(cppPoseStamped.pose, PyObject_GetAttrString(pyPoseStamped, "_pose")));

            return pyPoseStamped;
        }

        geometry_msgs::msg::Twist PyTwist_AsTwist(PyObject* pyTwist){
            geometry_msgs::msg::Twist cppTwist = geometry_msgs::msg::Twist();

            cppTwist.linear = PyVector3_AsVector3(PyObject_GetAttrString(pyTwist, "_linear"));
            cppTwist.angular = PyVector3_AsVector3(PyObject_GetAttrString(pyTwist, "_angular"));

            return cppTwist;
        }

        PyObject* PyTwist_FromTwist(const geometry_msgs::msg::Twist& cppTwist, PyObject* pyTwist = NULL) {
            if (pyTwist == NULL) {
                PyObject* twist_class = NULL;
                twist_class = PyObject_GetAttrString(module_GeometryMsgs, "Twist");

                pyTwist = PyObject_CallObject(twist_class, NULL);
                Py_XDECREF(twist_class);
            }

            PyObject_SetAttrString(pyTwist, "_linear", PyVector3_FromVector3(cppTwist.linear, PyObject_GetAttrString(pyTwist, "_linear")));
            PyObject_SetAttrString(pyTwist, "_angular", PyVector3_FromVector3(cppTwist.angular, PyObject_GetAttrString(pyTwist, "_angular")));

            return pyTwist;
        }

        geometry_msgs::msg::TwistStamped PyTwistStamped_AsTwistStamped(PyObject* pyTwistStamped){
            geometry_msgs::msg::TwistStamped cppTwistStamped = geometry_msgs::msg::TwistStamped();

            cppTwistStamped.header = stdMsgs->PyHeader_AsHeader(PyObject_GetAttrString(pyTwistStamped, "_header"));
            cppTwistStamped.twist = PyTwist_AsTwist(PyObject_GetAttrString(pyTwistStamped, "_twist"));

            return cppTwistStamped;
        }

        PyObject* PyTwistStamped_FromTwistStamped(const geometry_msgs::msg::TwistStamped& cppTwistStamped, PyObject* pyTwistStamped = NULL) {
            if (pyTwistStamped == NULL) {
                PyObject* twist_stamped_class = NULL;
                twist_stamped_class = PyObject_GetAttrString(module_GeometryMsgs, "TwistStamped");

                pyTwistStamped = PyObject_CallObject(twist_stamped_class, NULL);
                Py_XDECREF(twist_stamped_class);
            }

            PyObject_SetAttrString(pyTwistStamped, "_header", stdMsgs->PyHeader_FromHeader(cppTwistStamped.header, PyObject_GetAttrString(pyTwistStamped, "_header")));
            PyObject_SetAttrString(pyTwistStamped, "_twist", PyTwist_FromTwist(cppTwistStamped.twist, PyObject_GetAttrString(pyTwistStamped, "_twist")));

            return pyTwistStamped;
        }
};

GeometryMsgs* GeometryMsgs::pinstance_{nullptr};
std::mutex GeometryMsgs::mutex_;

GeometryMsgs* GeometryMsgs::GetInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr) {
        pinstance_ = new GeometryMsgs();
    }
    return pinstance_;
}

#endif // GEOMETRY_MSGS_PY_WRAPPER_HPP