#ifndef PYIF_GEOMETRY_MSGS_HPP
#define PYIF_GEOMETRY_MSGS_HPP

#include "python_interface.hpp"

#include "std_msgs.hpp"

namespace pyif {

geometry_msgs::msg::Vector3 GeoMsgs::PyVector3_AsVector3(PyObject* py_vector3) {
    geometry_msgs::msg::Vector3 cpp_vector3 = geometry_msgs::msg::Vector3();

    cpp_vector3.x = PyFloat_AsDouble(PyObject_GetAttrString(py_vector3, "_x"));
    cpp_vector3.y = PyFloat_AsDouble(PyObject_GetAttrString(py_vector3, "_y"));
    cpp_vector3.z = PyFloat_AsDouble(PyObject_GetAttrString(py_vector3, "_z"));

    return cpp_vector3;
}

PyObject* GeoMsgs::PyVector3_FromVector3(const geometry_msgs::msg::Vector3& cpp_vector3, PyObject* py_vector3 = NULL) {
    if (py_vector3 == NULL) {
        PyObject* vector3_class = PyMap::GetFunction("geometry_msgs.msg", "Vector3");

        py_vector3 = PyObject_CallObject(vector3_class, NULL);
        Py_XDECREF(vector3_class);
    }
    
    PyObject_SetAttrString(py_vector3, "_x", PyFloat_FromDouble(cpp_vector3.x));
    PyObject_SetAttrString(py_vector3, "_y", PyFloat_FromDouble(cpp_vector3.y));
    PyObject_SetAttrString(py_vector3, "_z", PyFloat_FromDouble(cpp_vector3.z));

    return py_vector3;
}

geometry_msgs::msg::Point GeoMsgs::PyPoint_AsPoint(PyObject* py_point){
    geometry_msgs::msg::Point cppPoint = geometry_msgs::msg::Point();

    cppPoint.x = PyFloat_AsDouble(PyObject_GetAttrString(py_point, "_x"));
    cppPoint.y = PyFloat_AsDouble(PyObject_GetAttrString(py_point, "_y"));
    cppPoint.z = PyFloat_AsDouble(PyObject_GetAttrString(py_point, "_z"));

    return cppPoint;
}

PyObject* GeoMsgs::PyPoint_FromPoint(const geometry_msgs::msg::Point& cppPoint, PyObject* py_point = NULL) {
    if (py_point == NULL) {
        PyObject* point_class = NULL;
        point_class = PyMap::GetFunction("geometry_msgs.msg", "Point");

        py_point = PyObject_CallObject(point_class, NULL);
        Py_XDECREF(point_class);
    }
    
    PyObject_SetAttrString(py_point, "_x", PyFloat_FromDouble(cppPoint.x));
    PyObject_SetAttrString(py_point, "_y", PyFloat_FromDouble(cppPoint.y));
    PyObject_SetAttrString(py_point, "_z", PyFloat_FromDouble(cppPoint.z));

    return py_point;
}

geometry_msgs::msg::Quaternion GeoMsgs::PyOrientation_AsOrientation(PyObject* py_orientation) {
    geometry_msgs::msg::Quaternion cpp_orientation = geometry_msgs::msg::Quaternion();

    cpp_orientation.x = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_x"));
    cpp_orientation.y = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_y"));
    cpp_orientation.z = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_z"));
    cpp_orientation.w = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_w"));

    return cpp_orientation;
}

PyObject* GeoMsgs::PyOrientation_FromOrientation(const geometry_msgs::msg::Quaternion& cpp_orientation, PyObject* py_orientation = NULL) {
    if (py_orientation == NULL) {
        PyObject* quaternion_class = NULL;
        quaternion_class = PyMap::GetFunction("geometry_msgs.msg", "Quaternion");

        py_orientation = PyObject_CallObject(quaternion_class, NULL);
        Py_XDECREF(quaternion_class);
    }
    
    PyObject_SetAttrString(py_orientation, "_x", PyFloat_FromDouble(cpp_orientation.x));
    PyObject_SetAttrString(py_orientation, "_y", PyFloat_FromDouble(cpp_orientation.y));
    PyObject_SetAttrString(py_orientation, "_z", PyFloat_FromDouble(cpp_orientation.z));
    PyObject_SetAttrString(py_orientation, "_w", PyFloat_FromDouble(cpp_orientation.w));
    
    return py_orientation;
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

} // namespace pyif

#endif // PYIF_GEOMETRY_MSGS_HPP