#ifndef NAV_MSGS_PY_WRAPPER_HPP
#define NAV_MSGS_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN


#include "std_msgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


class NavMsgs {
    private:
        PyObject* module_NavMsgs = NULL;
        StdMsgs* stdMsgs;
        GeometryMsgs* geometryMsgs;

        static NavMsgs* pinstance_;
        static std::mutex mutex_;

    protected:
        NavMsgs() {
            if (module_NavMsgs == NULL) {
                module_NavMsgs = PyImport_ImportModule("nav_msgs.msg");
                if (module_NavMsgs == NULL) {
                    std::cout << "module_NavMsgs == NULL" << std::endl;
                }
            }

            stdMsgs = StdMsgs::GetInstance();
            geometryMsgs = GeometryMsgs::GetInstance();
        }

        ~NavMsgs() {
            Py_XDECREF(module_NavMsgs);
        }

    public:
        NavMsgs(NavMsgs& other) = delete;
        void operator=(const NavMsgs&) = delete;

        static NavMsgs* GetInstance();

        nav_msgs::msg::MapMetaData PyMapMetaData_AsMapMetaData(PyObject* pyMapMetaData) {
            nav_msgs::msg::MapMetaData cppMapMetaData = nav_msgs::msg::MapMetaData();

            cppMapMetaData.map_load_time =  stdMsgs->PyStamp_AsStamp(PyObject_GetAttrString(pyMapMetaData, "_map_load_time"));
            cppMapMetaData.resolution = PyFloat_AsDouble(PyObject_GetAttrString(pyMapMetaData, "_resolution"));
            cppMapMetaData.width = PyLong_AsUnsignedLong(PyObject_GetAttrString(pyMapMetaData, "_width"));
            cppMapMetaData.height = PyLong_AsUnsignedLong(PyObject_GetAttrString(pyMapMetaData, "_height"));
            cppMapMetaData.origin = geometryMsgs->PyPose_AsPose(PyObject_GetAttrString(pyMapMetaData, "_origin"));

            return cppMapMetaData;
        }

        PyObject* PyMapMetaData_FromMapMetaData(const nav_msgs::msg::MapMetaData& cppMapMetaData, PyObject* pyMapMetaData = NULL) {
            if (pyMapMetaData == NULL) {
                PyObject* time_class = PyObject_GetAttrString(module_NavMsgs, "MapMetaData");

                pyMapMetaData = PyObject_CallObject(time_class, NULL);
                Py_XDECREF(time_class);
            }
            
            PyObject_SetAttrString(pyMapMetaData, "_map_load_time", stdMsgs->PyStamp_FromStamp(cppMapMetaData.map_load_time, PyObject_GetAttrString(pyMapMetaData, "_map_load_time")));
            PyObject_SetAttrString(pyMapMetaData, "_resolution", PyFloat_FromDouble(cppMapMetaData.resolution));
            PyObject_SetAttrString(pyMapMetaData, "_width", PyLong_FromUnsignedLong(cppMapMetaData.width));
            PyObject_SetAttrString(pyMapMetaData, "_height", PyLong_FromUnsignedLong(cppMapMetaData.height));
            PyObject_SetAttrString(pyMapMetaData, "_origin", geometryMsgs->PyPose_FromPose(cppMapMetaData.origin, PyObject_GetAttrString(pyMapMetaData, "_origin")));
            
            return pyMapMetaData;
        }

        nav_msgs::msg::OccupancyGrid PyOccupancyGrid_AsOccupancyGrid(PyObject* pyOccupancyGrid) {
            nav_msgs::msg::OccupancyGrid cppOccupancyGrid = nav_msgs::msg::OccupancyGrid();

            cppOccupancyGrid.header = stdMsgs->PyHeader_AsHeader(PyObject_GetAttrString(pyOccupancyGrid, "_header"));
            cppOccupancyGrid.info = PyMapMetaData_AsMapMetaData(PyObject_GetAttrString(pyOccupancyGrid, "_info"));
            
            PyObject* pyData = PyObject_GetAttrString(pyOccupancyGrid, "_data");
            Py_ssize_t size = PyList_Size(pyData);

            for (Py_ssize_t i = 0; i < size; ++i) {
                PyObject* pyLong= PyList_GetItem(pyData, i);
                cppOccupancyGrid.data[i] = PyLong_AsLong(pyLong);
            }

            return cppOccupancyGrid;
        }

        PyObject* PyOccupancyGrid_FromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& cppOccupancyGrid, PyObject* pyOccupancyGrid = NULL) {
            if (pyOccupancyGrid == NULL) {
                PyObject* occupancyGrid_class = PyObject_GetAttrString(module_NavMsgs, "OccupancyGrid");

                pyOccupancyGrid = PyObject_CallObject(occupancyGrid_class, NULL);
                Py_XDECREF(occupancyGrid_class);
            }

            PyObject_SetAttrString(pyOccupancyGrid, "_header", stdMsgs->PyHeader_FromHeader(cppOccupancyGrid.header, PyObject_GetAttrString(pyOccupancyGrid, "_header")));
            PyObject_SetAttrString(pyOccupancyGrid, "_info", PyMapMetaData_FromMapMetaData(cppOccupancyGrid.info, PyObject_GetAttrString(pyOccupancyGrid, "_info")));

            int size = cppOccupancyGrid.data.size();
            PyObject* pyData = PyList_New((Py_ssize_t) size);

            for (int i = 0; i < size; i++) {
                PyObject* pyLong = PyLong_FromLong(cppOccupancyGrid.data[i]);
                PyList_SetItem(pyData, i, pyLong);
            }

            PyObject_SetAttrString(pyOccupancyGrid, "_data", pyData);

            return pyOccupancyGrid;
        }

        nav_msgs::msg::Path PyPath_AsPath(PyObject* pyPath) {
            nav_msgs::msg::Path cppPath = nav_msgs::msg::Path();

            cppPath.header = stdMsgs->PyHeader_AsHeader(PyObject_GetAttrString(pyPath, "_header"));

            PyObject* pyPoses = PyObject_GetAttrString(pyPath, "_poses");
            Py_ssize_t size = PyList_Size(pyPoses);

            for (Py_ssize_t i = 0; i < size; ++i) {
                PyObject* pyPoseStamped = PyList_GetItem(pyPoses, i);
                cppPath.poses[i] = geometryMsgs->PyPoseStamped_AsPoseStamped(pyPoseStamped);
            }

            return cppPath;
        }

        PyObject* PyPath_FromPath(const nav_msgs::msg::Path& cppPath, PyObject* pyPath = NULL) {
            if (pyPath == NULL) {
                PyObject* path_class = PyObject_GetAttrString(module_NavMsgs, "Path");

                pyPath = PyObject_CallObject(path_class, NULL);
                Py_XDECREF(path_class);
            }

            PyObject_SetAttrString(pyPath, "_header", stdMsgs->PyHeader_FromHeader(cppPath.header, PyObject_GetAttrString(pyPath, "_header")));

            Py_ssize_t size = cppPath.poses.size();
            PyObject* pyPoses = PyList_New(size);
            for (Py_ssize_t i = 0; i < size; ++i) {
                PyObject* pyPoseStamped = geometryMsgs->PyPoseStamped_FromPoseStamped(cppPath.poses[i]);
                PyList_SetItem(pyPoses, i, pyPoseStamped);
            }

            PyObject_SetAttrString(pyPath, "_poses", pyPoses);

            return pyPath;
        }
};

NavMsgs* NavMsgs::pinstance_{nullptr};
std::mutex NavMsgs::mutex_;

NavMsgs* NavMsgs::GetInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr) {
        pinstance_ = new NavMsgs();
    }
    return pinstance_;
}

#endif // NAV_MSGS_PY_WRAPPER_HPP