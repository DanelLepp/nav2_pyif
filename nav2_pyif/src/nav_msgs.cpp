#include "nav2_pyif/nav_msgs.hpp"

nav_msgs::msg::MapMetaData pyif::NavMsgs::PyMapMetaData_AsMapMetaData(PyObject* py_map_metadata) {
    nav_msgs::msg::MapMetaData cpp_map_metadata = nav_msgs::msg::MapMetaData();

    cpp_map_metadata.map_load_time =  StdMsgs::PyStamp_AsStamp(PyObject_GetAttrString(py_map_metadata, "_map_load_time"));
    cpp_map_metadata.resolution = PyFloat_AsDouble(PyObject_GetAttrString(py_map_metadata, "_resolution"));
    cpp_map_metadata.width = PyLong_AsUnsignedLong(PyObject_GetAttrString(py_map_metadata, "_width"));
    cpp_map_metadata.height = PyLong_AsUnsignedLong(PyObject_GetAttrString(py_map_metadata, "_height"));
    cpp_map_metadata.origin = GeoMsgs::PyPose_AsPose(PyObject_GetAttrString(py_map_metadata, "_origin"));

    return cpp_map_metadata;
}

PyObject* pyif::NavMsgs::PyMapMetaData_FromMapMetaData(const nav_msgs::msg::MapMetaData& cpp_map_metadata, PyObject* py_map_metadata) {
    if (py_map_metadata == NULL) {
        PyObject* time_class = PyMap::GetFunction("nav_msgs.msg", "MapMetaData");

        py_map_metadata = PyObject_CallObject(time_class, NULL);
        Py_XDECREF(time_class);
    }
    
    PyObject_SetAttrString(py_map_metadata, "_map_load_time", StdMsgs::PyStamp_FromStamp(cpp_map_metadata.map_load_time, PyObject_GetAttrString(py_map_metadata, "_map_load_time")));
    PyObject_SetAttrString(py_map_metadata, "_resolution", PyFloat_FromDouble(cpp_map_metadata.resolution));
    PyObject_SetAttrString(py_map_metadata, "_width", PyLong_FromUnsignedLong(cpp_map_metadata.width));
    PyObject_SetAttrString(py_map_metadata, "_height", PyLong_FromUnsignedLong(cpp_map_metadata.height));
    PyObject_SetAttrString(py_map_metadata, "_origin", GeoMsgs::PyPose_FromPose(cpp_map_metadata.origin, PyObject_GetAttrString(py_map_metadata, "_origin")));
    
    return py_map_metadata;
}

nav_msgs::msg::OccupancyGrid pyif::NavMsgs::PyOccupancyGrid_AsOccupancyGrid(PyObject* py_occupancy_grid) {
    nav_msgs::msg::OccupancyGrid cpp_occupancy_grid = nav_msgs::msg::OccupancyGrid();

    cpp_occupancy_grid.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(py_occupancy_grid, "_header"));
    cpp_occupancy_grid.info = pyif::NavMsgs::PyMapMetaData_AsMapMetaData(PyObject_GetAttrString(py_occupancy_grid, "_info"));
    
    PyObject* pyData = PyObject_GetAttrString(py_occupancy_grid, "_data");
    Py_ssize_t size = PyList_Size(pyData);

    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* pyLong= PyList_GetItem(pyData, i);
        cpp_occupancy_grid.data[i] = PyLong_AsLong(pyLong);
    }

    return cpp_occupancy_grid;
}

PyObject* pyif::NavMsgs::PyOccupancyGrid_FromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& cpp_occupancy_grid, PyObject* py_occupancy_grid) {
    if (py_occupancy_grid == NULL) {
        PyObject* occupancyGrid_class = PyMap::GetFunction("nav_msgs.msg", "OccupancyGrid");

        py_occupancy_grid = PyObject_CallObject(occupancyGrid_class, NULL);
        Py_XDECREF(occupancyGrid_class);
    }

    PyObject_SetAttrString(py_occupancy_grid, "_header", StdMsgs::PyHeader_FromHeader(cpp_occupancy_grid.header, PyObject_GetAttrString(py_occupancy_grid, "_header")));
    PyObject_SetAttrString(py_occupancy_grid, "_info", pyif::NavMsgs::PyMapMetaData_FromMapMetaData(cpp_occupancy_grid.info, PyObject_GetAttrString(py_occupancy_grid, "_info")));

    int size = cpp_occupancy_grid.data.size();
    PyObject* pyData = PyList_New((Py_ssize_t) size);

    for (int i = 0; i < size; i++) {
        PyObject* pyLong = PyLong_FromLong(cpp_occupancy_grid.data[i]);
        PyList_SetItem(pyData, i, pyLong);
    }

    PyObject_SetAttrString(py_occupancy_grid, "_data", pyData);

    return py_occupancy_grid;
}

nav_msgs::msg::Path pyif::NavMsgs::PyPath_AsPath(PyObject* py_path) {
    nav_msgs::msg::Path cpp_path = nav_msgs::msg::Path();

    cpp_path.header = StdMsgs::PyHeader_AsHeader(PyObject_GetAttrString(py_path, "_header"));

    PyObject* py_poses = PyObject_GetAttrString(py_path, "_poses");
    Py_ssize_t size = PyList_Size(py_poses);

    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* py_pose_stamped = PyList_GetItem(py_poses, i);
        cpp_path.poses[i] = GeoMsgs::PyPoseStamped_AsPoseStamped(py_pose_stamped);
    }

    return cpp_path;
}

PyObject* pyif::NavMsgs::PyPath_FromPath(const nav_msgs::msg::Path& cpp_path, PyObject* py_path) {
    if (py_path == NULL) {
        PyObject* path_class = PyMap::GetFunction("nav_msgs.msg", "Path");

        if (path_class == NULL) {
            std::cout << "path_class == NULL" << std::endl;
            return NULL;
        }

        py_path = PyObject_CallObject(path_class, NULL);

        if (py_path == NULL){
            std::cout << "py_path == NULL" << std::endl;
            return NULL;
        }

        Py_XDECREF(path_class);
    }

    PyObject_SetAttrString(py_path, "_header", StdMsgs::PyHeader_FromHeader(cpp_path.header, PyObject_GetAttrString(py_path, "_header")));

    Py_ssize_t size = cpp_path.poses.size();
    PyObject* py_poses = PyList_New(size);
    for (Py_ssize_t i = 0; i < size; i++) {
        PyObject* py_pose_stamped = GeoMsgs::PyPoseStamped_FromPoseStamped(cpp_path.poses[i], NULL);
        PyList_SetItem(py_poses, i, py_pose_stamped);
    }
    PyObject_SetAttrString(py_path, "_poses", py_poses);

    return py_path;
}