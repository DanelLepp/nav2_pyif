#ifndef PYIF_GEOMETRY_MSGS_HPP
#define PYIF_GEOMETRY_MSGS_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "python_interface.hpp"
#include "std_msgs.hpp"

namespace pyif {

class GeoMsgs {
    public:
        static geometry_msgs::msg::Vector3 PyVector3_AsVector3(PyObject* py_vector3);

        static PyObject* PyVector3_FromVector3(const geometry_msgs::msg::Vector3& cpp_vector3, PyObject* py_vector3);

        static geometry_msgs::msg::Point PyPoint_AsPoint(PyObject* py_point);

        static PyObject* PyPoint_FromPoint(const geometry_msgs::msg::Point& cppPoint, PyObject* py_point);

        static geometry_msgs::msg::Quaternion PyOrientation_AsOrientation(PyObject* py_orientation);

        static PyObject* PyOrientation_FromOrientation(const geometry_msgs::msg::Quaternion& cpp_orientation, PyObject* py_orientation);

        static geometry_msgs::msg::Pose PyPose_AsPose(PyObject* py_pose);

        static PyObject* PyPose_FromPose(const geometry_msgs::msg::Pose& cpp_pose, PyObject* py_pose);

        static geometry_msgs::msg::PoseStamped PyPoseStamped_AsPoseStamped(PyObject* py_pose_stamped);

        static PyObject* PyPoseStamped_FromPoseStamped(const geometry_msgs::msg::PoseStamped& cpp_pose_stamped, PyObject* py_pose_stamped);

        static geometry_msgs::msg::Twist PyTwist_AsTwist(PyObject* py_twist);

        static PyObject* PyTwist_FromTwist(const geometry_msgs::msg::Twist& cpp_twist, PyObject* py_twist);

        static geometry_msgs::msg::TwistStamped PyTwistStamped_AsTwistStamped(PyObject* py_twist_stamped);

        static PyObject* PyTwistStamped_FromTwistStamped(const geometry_msgs::msg::TwistStamped& cpp_twist_stamped, PyObject* py_twist_stamped);
};

} // namespace pyif

#endif // PYIF_GEOMETRY_MSGS_HPP