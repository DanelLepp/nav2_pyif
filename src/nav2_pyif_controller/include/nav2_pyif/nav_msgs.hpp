#ifndef PYIF_NAV_MSGS_HPP
#define PYIF_NAV_MSGS_HPP

#include "python_interface.hpp"
#include "std_msgs.hpp"
#include "geometry_msgs.hpp"

namespace pyif {

nav_msgs::msg::MapMetaData NavMsgs::PyMapMetaData_AsMapMetaData(PyObject* pyMapMetaData) {
    nav_msgs::msg::MapMetaData cppMapMetaData = nav_msgs::msg::MapMetaData();

    cppMapMetaData.map_load_time =  StdMsgs::PyStamp_AsStamp(PyObject_GetAttrString(pyMapMetaData, "_map_load_time"));
    cppMapMetaData.resolution = PyFloat_AsDouble(PyObject_GetAttrString(pyMapMetaData, "_resolution"));
    cppMapMetaData.width = PyLong_AsUnsignedLong(PyObject_GetAttrString(pyMapMetaData, "_width"));
    cppMapMetaData.height = PyLong_AsUnsignedLong(PyObject_GetAttrString(pyMapMetaData, "_height"));
    cppMapMetaData.origin = GeoMsgs::PyPose_AsPose(PyObject_GetAttrString(pyMapMetaData, "_origin"));

    return cppMapMetaData;
}

PyObject* NavMsgs::PyMapMetaData_FromMapMetaData(const nav_msgs::msg::MapMetaData& cppMapMetaData, PyObject* pyMapMetaData = NULL) {
    if (pyMapMetaData == NULL) {
        PyObject* time_class = PyMap::GetFunction("nav_msgs.msg", "MapMetaData");

        pyMapMetaData = PyObject_CallObject(time_class, NULL);
        Py_XDECREF(time_class);
    }
    
    PyObject_SetAttrString(pyMapMetaData, "_map_load_time", StdMsgs::PyStamp_FromStamp(cppMapMetaData.map_load_time, PyObject_GetAttrString(pyMapMetaData, "_map_load_time")));
    PyObject_SetAttrString(pyMapMetaData, "_resolution", PyFloat_FromDouble(cppMapMetaData.resolution));
    PyObject_SetAttrString(pyMapMetaData, "_width", PyLong_FromUnsignedLong(cppMapMetaData.width));
    PyObject_SetAttrString(pyMapMetaData, "_height", PyLong_FromUnsignedLong(cppMapMetaData.height));
    PyObject_SetAttrString(pyMapMetaData, "_origin", GeoMsgs::PyPose_FromPose(cppMapMetaData.origin, PyObject_GetAttrString(pyMapMetaData, "_origin")));
    
    return pyMapMetaData;
}

nav_msgs::msg::OccupancyGrid NavMsgs::PyOccupancyGrid_AsOccupancyGrid(PyObject* pyOccupancyGrid) {
    nav_msgs::msg::OccupancyGrid cppOccupancyGrid = nav_msgs::msg::OccupancyGrid();

    cppOccupancyGrid.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(pyOccupancyGrid, "_header"));
    cppOccupancyGrid.info = NavMsgs::PyMapMetaData_AsMapMetaData(PyObject_GetAttrString(pyOccupancyGrid, "_info"));
    
    PyObject* pyData = PyObject_GetAttrString(pyOccupancyGrid, "_data");
    Py_ssize_t size = PyList_Size(pyData);

    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* pyLong= PyList_GetItem(pyData, i);
        cppOccupancyGrid.data[i] = PyLong_AsLong(pyLong);
    }

    return cppOccupancyGrid;
}

PyObject* NavMsgs::PyOccupancyGrid_FromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& cppOccupancyGrid, PyObject* pyOccupancyGrid = NULL) {
    if (pyOccupancyGrid == NULL) {
        PyObject* occupancyGrid_class = PyMap::GetFunction("nav_msgs.msg", "OccupancyGrid");

        pyOccupancyGrid = PyObject_CallObject(occupancyGrid_class, NULL);
        Py_XDECREF(occupancyGrid_class);
    }

    PyObject_SetAttrString(pyOccupancyGrid, "_header", StdMsgs::PyHeader_FromHeader(cppOccupancyGrid.header, PyObject_GetAttrString(pyOccupancyGrid, "_header")));
    PyObject_SetAttrString(pyOccupancyGrid, "_info", NavMsgs::PyMapMetaData_FromMapMetaData(cppOccupancyGrid.info, PyObject_GetAttrString(pyOccupancyGrid, "_info")));

    int size = cppOccupancyGrid.data.size();
    PyObject* pyData = PyList_New((Py_ssize_t) size);

    for (int i = 0; i < size; i++) {
        PyObject* pyLong = PyLong_FromLong(cppOccupancyGrid.data[i]);
        PyList_SetItem(pyData, i, pyLong);
    }

    PyObject_SetAttrString(pyOccupancyGrid, "_data", pyData);

    return pyOccupancyGrid;
}

nav_msgs::msg::Path NavMsgs::PyPath_AsPath(PyObject* pyPath) {
    nav_msgs::msg::Path cppPath = nav_msgs::msg::Path();

    cppPath.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(pyPath, "_header"));

    PyObject* pyPoses = PyObject_GetAttrString(pyPath, "_poses");
    Py_ssize_t size = PyList_Size(pyPoses);

    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* pyPoseStamped = PyList_GetItem(pyPoses, i);
        cppPath.poses[i] = GeoMsgs::PyPoseStamped_AsPoseStamped(pyPoseStamped);
    }

    return cppPath;
}

PyObject* NavMsgs::PyPath_FromPath(const nav_msgs::msg::Path& cppPath, PyObject* pyPath = NULL) {
    if (pyPath == NULL) {
        PyObject* path_class = PyMap::GetFunction("nav_msgs.msg", "Path");

        if (path_class == NULL) {
            std::cout << "path_class == NULL" << std::endl;
            return NULL;
        }

        pyPath = PyObject_CallObject(path_class, NULL);

        if (pyPath == NULL){
            std::cout << "pyPath == NULL" << std::endl;
            return NULL;
        }

        Py_XDECREF(path_class);
    }

    PyObject_SetAttrString(pyPath, "_header", StdMsgs::PyHeader_FromHeader(cppPath.header, PyObject_GetAttrString(pyPath, "_header")));

    Py_ssize_t size = cppPath.poses.size();
    PyObject* pyPoses = PyList_New(size);
    for (Py_ssize_t i = 0; i < size; i++) {
        PyObject* pyPoseStamped = GeoMsgs::PyPoseStamped_FromPoseStamped(cppPath.poses[i]);
        PyList_SetItem(pyPoses, i, pyPoseStamped);
    }
    PyObject_SetAttrString(pyPath, "_poses", pyPoses);

    return pyPath;
}

}; // namespace pyif

#endif // PYIF_NAV_MSGS_HPP