#ifndef PYIF_NAV_MSGS_HPP
#define PYIF_NAV_MSGS_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "python_interface.hpp"
#include "std_msgs.hpp"
#include "geo_msgs.hpp"

namespace pyif {

class NavMsgs {
    public:
        static nav_msgs::msg::MapMetaData PyMapMetaData_AsMapMetaData(PyObject* py_map_metadata);

        static PyObject* PyMapMetaData_FromMapMetaData(const nav_msgs::msg::MapMetaData& cpp_map_metadata, PyObject* py_map_metadata);

        static nav_msgs::msg::OccupancyGrid PyOccupancyGrid_AsOccupancyGrid(PyObject* py_occupancy_grid);

        static PyObject* PyOccupancyGrid_FromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& cpp_occupancy_grid, PyObject* py_occupancy_grid);

        static nav_msgs::msg::Path PyPath_AsPath(PyObject* py_path);

        static PyObject* PyPath_FromPath(const nav_msgs::msg::Path& cpp_path, PyObject* py_path);
};

} // namespace pyif

#endif // PYIF_NAV_MSGS_HPP