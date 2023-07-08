#include "nav2_pyif/geo_msgs.hpp"

#define assertm(exp, msg) assert(((void)msg, exp))

geometry_msgs::msg::Vector3 pyif::GeoMsgs::PyVector3_AsVector3(PyObject* py_vector3) {
    geometry_msgs::msg::Vector3 cpp_vector3;
    cpp_vector3.x = PyFloat_AsDouble(PyObject_GetAttrString(py_vector3, "_x"));
    cpp_vector3.y = PyFloat_AsDouble(PyObject_GetAttrString(py_vector3, "_y"));
    cpp_vector3.z = PyFloat_AsDouble(PyObject_GetAttrString(py_vector3, "_z"));

    return cpp_vector3;
}

// TODO: Replicate this for all geometry_msgs
PyObject* pyif::GeoMsgs::PyVector3_FromVector3(const geometry_msgs::msg::Vector3& cpp_vector3, PyObject* py_vector3) {
    if (py_vector3 == NULL) {
        PyObject* vector3_class = PyMap::GetFunction("geometry_msgs.msg", "Vector3");

        py_vector3 = PyObject_CallObject(vector3_class, NULL);
        Py_XDECREF(vector3_class);
    }

    assertm(py_vector3 != NULL, "py_vector3 is NULL");

    PyObject_SetAttrString(py_vector3, "_x", PyFloat_FromDouble(cpp_vector3.x));
    PyObject_SetAttrString(py_vector3, "_y", PyFloat_FromDouble(cpp_vector3.y));
    PyObject_SetAttrString(py_vector3, "_z", PyFloat_FromDouble(cpp_vector3.z));

    return py_vector3;
}

geometry_msgs::msg::Point pyif::GeoMsgs::PyPoint_AsPoint(PyObject* py_point){
    geometry_msgs::msg::Point cppPoint = geometry_msgs::msg::Point();

    cppPoint.x = PyFloat_AsDouble(PyObject_GetAttrString(py_point, "_x"));
    cppPoint.y = PyFloat_AsDouble(PyObject_GetAttrString(py_point, "_y"));
    cppPoint.z = PyFloat_AsDouble(PyObject_GetAttrString(py_point, "_z"));

    return cppPoint;
}

PyObject* pyif::GeoMsgs::PyPoint_FromPoint(const geometry_msgs::msg::Point& cppPoint, PyObject* py_point) {
    if (py_point == NULL) {
        PyObject* point_class;
        point_class = PyMap::GetFunction("geometry_msgs.msg", "Point");

        py_point = PyObject_CallObject(point_class, NULL);
        Py_XDECREF(point_class);
    }
    
    PyObject_SetAttrString(py_point, "_x", PyFloat_FromDouble(cppPoint.x));
    PyObject_SetAttrString(py_point, "_y", PyFloat_FromDouble(cppPoint.y));
    PyObject_SetAttrString(py_point, "_z", PyFloat_FromDouble(cppPoint.z));

    return py_point;
}

geometry_msgs::msg::Quaternion pyif::GeoMsgs::PyOrientation_AsOrientation(PyObject* py_orientation) {
    geometry_msgs::msg::Quaternion cpp_orientation = geometry_msgs::msg::Quaternion();

    cpp_orientation.x = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_x"));
    cpp_orientation.y = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_y"));
    cpp_orientation.z = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_z"));
    cpp_orientation.w = PyFloat_AsDouble(PyObject_GetAttrString(py_orientation, "_w"));

    return cpp_orientation;
}

PyObject* pyif::GeoMsgs::PyOrientation_FromOrientation(const geometry_msgs::msg::Quaternion& cpp_orientation, PyObject* py_orientation) {
    if (py_orientation == NULL) {
        PyObject* quaternion_class;
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

geometry_msgs::msg::Pose pyif::GeoMsgs::PyPose_AsPose(PyObject* py_pose) {
    geometry_msgs::msg::Pose cpp_pose = geometry_msgs::msg::Pose();

    cpp_pose.position = PyPoint_AsPoint(PyObject_GetAttrString(py_pose, "_position"));
    cpp_pose.orientation = PyOrientation_AsOrientation(PyObject_GetAttrString(py_pose, "_orientation"));

    return cpp_pose;
}

PyObject* pyif::GeoMsgs::PyPose_FromPose(const geometry_msgs::msg::Pose& cpp_pose, PyObject* py_pose) {
    if (py_pose == NULL) {
        PyObject* pose_class;
        pose_class = PyMap::GetFunction("geometry_msgs.msg", "Pose");

        py_pose = PyObject_CallObject(pose_class, NULL);
        Py_XDECREF(pose_class);
    }
    
    PyObject_SetAttrString(py_pose, "_position", PyPoint_FromPoint(cpp_pose.position, PyObject_GetAttrString(py_pose, "_position")));
    PyObject_SetAttrString(py_pose, "_orientation", PyOrientation_FromOrientation(cpp_pose.orientation, PyObject_GetAttrString(py_pose, "_orientation")));

    return py_pose;
}

geometry_msgs::msg::PoseStamped pyif::GeoMsgs::PyPoseStamped_AsPoseStamped(PyObject* py_pose_stamped){
    geometry_msgs::msg::PoseStamped cpp_pose_stamped = geometry_msgs::msg::PoseStamped();

    cpp_pose_stamped.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(py_pose_stamped, "_header"));
    cpp_pose_stamped.pose = PyPose_AsPose(PyObject_GetAttrString(py_pose_stamped, "_pose"));


    return cpp_pose_stamped;
}

PyObject* pyif::GeoMsgs::PyPoseStamped_FromPoseStamped(const geometry_msgs::msg::PoseStamped& cpp_pose_stamped, PyObject* py_pose_stamped) {
    if (py_pose_stamped == NULL) {
        PyObject* pose_stamped_class;
        pose_stamped_class = PyMap::GetFunction("geometry_msgs.msg", "PoseStamped");

        py_pose_stamped = PyObject_CallObject(pose_stamped_class, NULL);
        Py_XDECREF(pose_stamped_class);
    }
    
    PyObject_SetAttrString(py_pose_stamped, "_header", StdMsgs::PyHeader_FromHeader(cpp_pose_stamped.header, PyObject_GetAttrString(py_pose_stamped, "_header")));
    PyObject_SetAttrString(py_pose_stamped, "_pose", PyPose_FromPose(cpp_pose_stamped.pose, PyObject_GetAttrString(py_pose_stamped, "_pose")));

    return py_pose_stamped;
}

geometry_msgs::msg::Twist pyif::GeoMsgs::PyTwist_AsTwist(PyObject* py_twist){
    geometry_msgs::msg::Twist cpp_twist = geometry_msgs::msg::Twist();

    cpp_twist.linear = PyVector3_AsVector3(PyObject_GetAttrString(py_twist, "_linear"));
    cpp_twist.angular = PyVector3_AsVector3(PyObject_GetAttrString(py_twist, "_angular"));

    return cpp_twist;
}

PyObject* pyif::GeoMsgs::PyTwist_FromTwist(const geometry_msgs::msg::Twist& cpp_twist, PyObject* py_twist) {
    if (py_twist == NULL) {
        PyObject* twist_class = pyif::PyMap::GetFunction("geometry_msgs.msg", "Twist");

        py_twist = PyObject_CallObject(twist_class, NULL);
        Py_XDECREF(twist_class);
    }

    PyObject_SetAttrString(py_twist, "_linear", PyVector3_FromVector3(cpp_twist.linear, PyObject_GetAttrString(py_twist, "_linear")));
    PyObject_SetAttrString(py_twist, "_angular", PyVector3_FromVector3(cpp_twist.angular, PyObject_GetAttrString(py_twist, "_angular")));

    return py_twist;
}

geometry_msgs::msg::TwistStamped pyif::GeoMsgs::PyTwistStamped_AsTwistStamped(PyObject* py_twist_stamped){
    geometry_msgs::msg::TwistStamped cpp_twist_stamped = geometry_msgs::msg::TwistStamped();

    if (py_twist_stamped != NULL) {
        cpp_twist_stamped.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(py_twist_stamped, "_header"));
        cpp_twist_stamped.twist = PyTwist_AsTwist(PyObject_GetAttrString(py_twist_stamped, "_twist"));
    }

    return cpp_twist_stamped;
}

PyObject* pyif::GeoMsgs::PyTwistStamped_FromTwistStamped(const geometry_msgs::msg::TwistStamped& cpp_twist_stamped, PyObject* py_twist_stamped) {
    if (py_twist_stamped == NULL) {
        PyObject* twist_stamped_class;
        twist_stamped_class = pyif::PyMap::GetFunction("geometry_msgs.msg", "TwistStamped");

        py_twist_stamped = PyObject_CallObject(twist_stamped_class, NULL);
        Py_XDECREF(twist_stamped_class);
    }

    PyObject_SetAttrString(py_twist_stamped, "_header", StdMsgs::StdMsgs::PyHeader_FromHeader(cpp_twist_stamped.header, PyObject_GetAttrString(py_twist_stamped, "_header")));
    PyObject_SetAttrString(py_twist_stamped, "_twist", PyTwist_FromTwist(cpp_twist_stamped.twist, PyObject_GetAttrString(py_twist_stamped, "_twist")));

    return py_twist_stamped;
}