#ifndef GEOMETRY_MSGS_PY_WRAPPER_HPP
#define GEOMETRY_MSGS_PY_WRAPPER_HPP

#include "python_interface.hpp"
#include "std_msgs.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace pyif {

geometry_msgs::msg::Vector3 GeoMsgs::PyVector3_AsVector3(PyObject* pyVector3) {
    geometry_msgs::msg::Vector3 cppVector3 = geometry_msgs::msg::Vector3();

    cppVector3.x = PyFloat_AsDouble(PyObject_GetAttrString(pyVector3, "_x"));
    cppVector3.y = PyFloat_AsDouble(PyObject_GetAttrString(pyVector3, "_y"));
    cppVector3.z = PyFloat_AsDouble(PyObject_GetAttrString(pyVector3, "_z"));

    return cppVector3;
}

PyObject* GeoMsgs::PyVector3_FromVector3(const geometry_msgs::msg::Vector3& cppVector3, PyObject* pyVector3 = NULL) {
    if (pyVector3 == NULL) {
        PyObject* vector3_class = PyMap::GetFunction("geometry_msgs.msg", "Vector3");

        pyVector3 = PyObject_CallObject(vector3_class, NULL);
        Py_XDECREF(vector3_class);
    }
    
    PyObject_SetAttrString(pyVector3, "_x", PyFloat_FromDouble(cppVector3.x));
    PyObject_SetAttrString(pyVector3, "_y", PyFloat_FromDouble(cppVector3.y));
    PyObject_SetAttrString(pyVector3, "_z", PyFloat_FromDouble(cppVector3.z));

    return pyVector3;
}

geometry_msgs::msg::Point GeoMsgs::PyPoint_AsPoint(PyObject* pyPoint){
    geometry_msgs::msg::Point cppPoint = geometry_msgs::msg::Point();

    cppPoint.x = PyFloat_AsDouble(PyObject_GetAttrString(pyPoint, "_x"));
    cppPoint.y = PyFloat_AsDouble(PyObject_GetAttrString(pyPoint, "_y"));
    cppPoint.z = PyFloat_AsDouble(PyObject_GetAttrString(pyPoint, "_z"));

    return cppPoint;
}

PyObject* GeoMsgs::PyPoint_FromPoint(const geometry_msgs::msg::Point& cppPoint, PyObject* pyPoint = NULL) {
    if (pyPoint == NULL) {
        PyObject* point_class = NULL;
        point_class = PyMap::GetFunction("geometry_msgs.msg", "Point");

        pyPoint = PyObject_CallObject(point_class, NULL);
        Py_XDECREF(point_class);
    }
    
    PyObject_SetAttrString(pyPoint, "_x", PyFloat_FromDouble(cppPoint.x));
    PyObject_SetAttrString(pyPoint, "_y", PyFloat_FromDouble(cppPoint.y));
    PyObject_SetAttrString(pyPoint, "_z", PyFloat_FromDouble(cppPoint.z));

    return pyPoint;
}

geometry_msgs::msg::Quaternion GeoMsgs::PyOrientation_AsOrientation(PyObject* pyOrientation) {
    geometry_msgs::msg::Quaternion cppOrientation = geometry_msgs::msg::Quaternion();

    cppOrientation.x = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_x"));
    cppOrientation.y = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_y"));
    cppOrientation.z = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_z"));
    cppOrientation.w = PyFloat_AsDouble(PyObject_GetAttrString(pyOrientation, "_w"));

    return cppOrientation;
}

PyObject* GeoMsgs::PyOrientation_FromOrientation(const geometry_msgs::msg::Quaternion& cppOrientation, PyObject* pyOrientation = NULL) {
    if (pyOrientation == NULL) {
        PyObject* quaternion_class = NULL;
        quaternion_class = PyMap::GetFunction("geometry_msgs.msg", "Quaternion");

        pyOrientation = PyObject_CallObject(quaternion_class, NULL);
        Py_XDECREF(quaternion_class);
    }
    
    PyObject_SetAttrString(pyOrientation, "_x", PyFloat_FromDouble(cppOrientation.x));
    PyObject_SetAttrString(pyOrientation, "_y", PyFloat_FromDouble(cppOrientation.y));
    PyObject_SetAttrString(pyOrientation, "_z", PyFloat_FromDouble(cppOrientation.z));
    PyObject_SetAttrString(pyOrientation, "_w", PyFloat_FromDouble(cppOrientation.w));
    
    return pyOrientation;
}

geometry_msgs::msg::Pose GeoMsgs::PyPose_AsPose(PyObject* pyPose) {
    geometry_msgs::msg::Pose cppPose = geometry_msgs::msg::Pose();

    cppPose.position = PyPoint_AsPoint(PyObject_GetAttrString(pyPose, "_position"));
    cppPose.orientation = PyOrientation_AsOrientation(PyObject_GetAttrString(pyPose, "_orientation"));

    return cppPose;
}

PyObject* GeoMsgs::PyPose_FromPose(const geometry_msgs::msg::Pose& cppPose, PyObject* pyPose = NULL) {
    if (pyPose == NULL) {
        PyObject* pose_class = NULL;
        pose_class = PyMap::GetFunction("geometry_msgs.msg", "Pose");

        pyPose = PyObject_CallObject(pose_class, NULL);
        Py_XDECREF(pose_class);
    }
    
    PyObject_SetAttrString(pyPose, "_position", PyPoint_FromPoint(cppPose.position, PyObject_GetAttrString(pyPose, "_position")));
    PyObject_SetAttrString(pyPose, "_orientation", PyOrientation_FromOrientation(cppPose.orientation, PyObject_GetAttrString(pyPose, "_orientation")));

    return pyPose;
}

geometry_msgs::msg::PoseStamped GeoMsgs::PyPoseStamped_AsPoseStamped(PyObject* pyPoseStamped){
    geometry_msgs::msg::PoseStamped cppPoseStamped = geometry_msgs::msg::PoseStamped();

    cppPoseStamped.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(pyPoseStamped, "_header"));
    cppPoseStamped.pose = PyPose_AsPose(PyObject_GetAttrString(pyPoseStamped, "_pose"));


    return cppPoseStamped;
}

PyObject* GeoMsgs::PyPoseStamped_FromPoseStamped(const geometry_msgs::msg::PoseStamped& cppPoseStamped, PyObject* pyPoseStamped = NULL) {
    if (pyPoseStamped == NULL) {
        PyObject* pose_stamped_class = NULL;
        pose_stamped_class = PyMap::GetFunction("geometry_msgs.msg", "PoseStamped");

        pyPoseStamped = PyObject_CallObject(pose_stamped_class, NULL);
        Py_XDECREF(pose_stamped_class);
    }
    
    PyObject_SetAttrString(pyPoseStamped, "_header", StdMsgs::PyHeader_FromHeader(cppPoseStamped.header, PyObject_GetAttrString(pyPoseStamped, "_header")));
    PyObject_SetAttrString(pyPoseStamped, "_pose", PyPose_FromPose(cppPoseStamped.pose, PyObject_GetAttrString(pyPoseStamped, "_pose")));

    return pyPoseStamped;
}

geometry_msgs::msg::Twist GeoMsgs::PyTwist_AsTwist(PyObject* pyTwist){
    geometry_msgs::msg::Twist cppTwist = geometry_msgs::msg::Twist();

    cppTwist.linear = PyVector3_AsVector3(PyObject_GetAttrString(pyTwist, "_linear"));
    cppTwist.angular = PyVector3_AsVector3(PyObject_GetAttrString(pyTwist, "_angular"));

    return cppTwist;
}

PyObject* GeoMsgs::PyTwist_FromTwist(const geometry_msgs::msg::Twist& cppTwist, PyObject* pyTwist = NULL) {
    if (pyTwist == NULL) {
        PyObject* twist_class = pyif::PyMap::GetFunction("geometry_msgs.msg", "Twist");

        pyTwist = PyObject_CallObject(twist_class, NULL);
        Py_XDECREF(twist_class);
    }

    PyObject_SetAttrString(pyTwist, "_linear", PyVector3_FromVector3(cppTwist.linear, PyObject_GetAttrString(pyTwist, "_linear")));
    PyObject_SetAttrString(pyTwist, "_angular", PyVector3_FromVector3(cppTwist.angular, PyObject_GetAttrString(pyTwist, "_angular")));

    return pyTwist;
}

geometry_msgs::msg::TwistStamped GeoMsgs::PyTwistStamped_AsTwistStamped(PyObject* pyTwistStamped){
    geometry_msgs::msg::TwistStamped cppTwistStamped = geometry_msgs::msg::TwistStamped();

    if (pyTwistStamped != NULL) {
        cppTwistStamped.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(pyTwistStamped, "_header"));
        cppTwistStamped.twist = PyTwist_AsTwist(PyObject_GetAttrString(pyTwistStamped, "_twist"));
    }

    return cppTwistStamped;
}

PyObject* GeoMsgs::PyTwistStamped_FromTwistStamped(const geometry_msgs::msg::TwistStamped& cppTwistStamped, PyObject* pyTwistStamped = NULL) {
    if (pyTwistStamped == NULL) {
        PyObject* twist_stamped_class = NULL;
        twist_stamped_class = pyif::PyMap::GetFunction("geometry_msgs.msg", "TwistStamped");

        pyTwistStamped = PyObject_CallObject(twist_stamped_class, NULL);
        Py_XDECREF(twist_stamped_class);
    }

    PyObject_SetAttrString(pyTwistStamped, "_header", StdMsgs::StdMsgs::PyHeader_FromHeader(cppTwistStamped.header, PyObject_GetAttrString(pyTwistStamped, "_header")));
    PyObject_SetAttrString(pyTwistStamped, "_twist", PyTwist_FromTwist(cppTwistStamped.twist, PyObject_GetAttrString(pyTwistStamped, "_twist")));

    return pyTwistStamped;
}

}; // namespace pyif

#endif // GEOMETRY_MSGS_PY_WRAPPER_HPP