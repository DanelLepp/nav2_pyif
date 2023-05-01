#include <iostream>

#include <dlfcn.h>
#include "python3.10/Python.h"
#include "python_wrappers/py_wrapper.hpp"
#include "python_wrappers/nav_msgs.hpp"
#include "python_wrappers/geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

void setPlan(const nav_msgs::msg::Path &path)
{
    PyObject *pyGlobalPath = PyPath_FromPath(path);
    if (pyGlobalPath == NULL)
    {
        std::cout << "pyGlobalPath == NULL" << std::endl;
        return;
    }

    PyObject *arguments = PyTuple_New(1);
    if (arguments == NULL)
    {
        std::cout << "arguments == NULL" << std::endl;
        return;
    }

    PyTuple_SetItem(arguments, 0, pyGlobalPath);

    PyObject *func_setPath = PyWrapper::GetFunction("artificial_potential_field.artificial_potential_field", "setPath");
    if (func_setPath == NULL)
    {
        std::cout << "func_setPath == NULL" << std::endl;
        return;
    }
    PyObject_CallObject(func_setPath, arguments);
}

geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose)
{

    nav_msgs::msg::OccupancyGrid occupancyGrid = nav_msgs::msg::OccupancyGrid();
    occupancyGrid.info.resolution = 1;
    occupancyGrid.info.width = 25;
    occupancyGrid.info.height = 50;
    std::vector<signed char> data;
    for (unsigned int y = 0; y < occupancyGrid.info.height; y++)
    {
        for (unsigned int x = 0; x < occupancyGrid.info.width; x++)
        {
            data.push_back((y == 10) ? 100 : 50);
        }
    }
    occupancyGrid.data = data;

    PyObject *pyOccupancyGrid = PyOccupancyGrid_FromOccupancyGrid(occupancyGrid);
    if (pyOccupancyGrid == NULL)
    {
        std::cout << "pyOccupancyGrid == NULL" << std::endl;
    }

    PyObject *pyPose = PyPoseStamped_FromPoseStamped(pose);
    if (pyPose == NULL)
    {
        std::cout << "pyPose == NULL" << std::endl;
    }

    // PyObject* pyTwist = PyTwist_FromTwist(twist);

    PyObject *arguments = PyTuple_New(2);
    PyTuple_SetItem(arguments, 0, pyOccupancyGrid);
    PyTuple_SetItem(arguments, 1, pyPose);
    // PyTuple_SetItem(arguments, 2, pyTwist);

    PyObject *func_getVelocity = PyWrapper::GetFunction("artificial_potential_field.artificial_potential_field", "computeVelocityCommands");

    if (func_getVelocity == NULL)
    {
        std::cout << "func_getVelocity == NULL" << std::endl;
    }

    PyObject *pyTwistStamped = PyObject_CallObject(func_getVelocity, arguments);

    if (pyTwistStamped == NULL)
    {
        std::cout << "pyTwistStamped == NULL" << std::endl;
    }

    auto twistStamped = PyTwistStamped_AsTwistStamped(pyTwistStamped);

    // Py_XDECREF(pyOccupancyGrid);
    // Py_XDECREF(pyPose);
    // Py_XDECREF(pyGlobalPath);
    // Py_XDECREF(arguments);
    // Py_XDECREF(func_getVelocity);
    // Py_XDECREF(value);

    // geometry_msgs::msg::TwistStamped twistStamped = apf_controller->getVelocity(costmap_msg, pose, globalPath);
    return twistStamped;
}

void Initialise()
{
    for (int i = 0; i < 100; i++)
    {
        auto start = std::chrono::system_clock::now();
        Py_Initialize();
        auto end = std::chrono::system_clock::now();
        std::cout << "init " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

        auto start2 = std::chrono::system_clock::now();
        Py_Finalize();
        auto end2 = std::chrono::system_clock::now();
        std::cout << " deinit " << std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - start2).count() << std::endl;
    }
}

void ModuleImport()
{
    for (int i = 0; i < 100; i++)
    {
        auto start3 = std::chrono::system_clock::now();
        PyImport_ImportModule("std_msgs.msg");
        auto end3 = std::chrono::system_clock::now();
        std::cout << " std_msgs " << std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - start3).count();

        auto start2 = std::chrono::system_clock::now();
        PyImport_ImportModule("geometry_msgs.msg");
        auto end2 = std::chrono::system_clock::now();
        std::cout << " geom_msgs " << std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - start2).count();

        auto start = std::chrono::system_clock::now();
        PyImport_ImportModule("nav_msgs.msg");
        auto end = std::chrono::system_clock::now();
        std::cout << " nav_msgs " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;


    }
}

void GetFunction()
{   
    std::vector<std::pair<std::string, std::string>> funcs;
    funcs.push_back({"artificial_potential_field.artificial_potential_field", "computeVelocityCommands"});
    PyWrapper::Init(funcs);
    for (int i = 0; i < 100; i++)
    {
        auto start = std::chrono::system_clock::now();
        PyWrapper::GetFunction("artificial_potential_field.artificial_potential_field", "computeVelocityCommands");
        auto end = std::chrono::system_clock::now();
        std::cout << " get func " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;
    }
    PyWrapper::DeInit();
}

void GetFunctionRaw()
{   
    std::pair<std::string, std::string> funcs = {"artificial_potential_field.artificial_potential_field", "computeVelocityCommands"};
    for (int i = 0; i < 100; i++)
    {
        auto start = std::chrono::system_clock::now();
        PyObject* pyModule = PyImport_ImportModule(funcs.first.c_str());
        PyObject_GetAttrString(pyModule, funcs.second.c_str());
        auto end = std::chrono::system_clock::now();
        std::cout << " get func " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;
    }
}

void ImportTest()
{
    //   ArtificialPotentialField* apfModule = ArtificialPotentialField::GetInstance();

    std::cout << "ImportTest " << std::endl;
    geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x = 15;
    pose.pose.position.y = 15;
    nav_msgs::msg::Path globalPath = nav_msgs::msg::Path();
    geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
    goal_pose.pose.position.x = 24;
    goal_pose.pose.position.y = 1;
    globalPath.poses = std::vector<geometry_msgs::msg::PoseStamped>{goal_pose};
    std::cout << "size " << globalPath.poses.size() << std::endl;

    setPlan(globalPath);
    computeVelocityCommands(pose);
}

// void ThroughPut() {
//   ArtificialPotentialField* apfModule = ArtificialPotentialField::GetInstance();

//   nav_msgs::msg::OccupancyGrid costmap_msg = nav_msgs::msg::OccupancyGrid();
//   costmap_msg.info.resolution = 1;
//   costmap_msg.info.width = 4;
//   costmap_msg.info.height = 4;
//   costmap_msg.data = std::vector<signed char>{10, 20, 30, 40, 11, 21, 31, 41, 12, 22, 32, 42, 13, 23, 33, 43};
//   geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
//   nav_msgs::msg::Path globalPath = nav_msgs::msg::Path();
//   geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
//   goal_pose.pose.position.x = 2;
//   goal_pose.pose.position.y = 2;
//   globalPath.poses = std::vector<geometry_msgs::msg::PoseStamped>{goal_pose};

//   // GeometryMsgs* geometryMsgs = GeometryMsgs::GetInstance();
//   // NavMsgs* navMsgs = NavMsgs::GetInstance();
//   for(int i = 0; i < 1000; i++) {
//     apfModule->getVelocity(costmap_msg, pose, globalPath);
//     // PyObject* arguments = PyTuple_New(3);;
//     // PyObject* pyOccupancyGrid = navMsgs->PyOccupancyGrid_FromOccupancyGrid(costmap_msg);
//     // PyObject* pyPose = geometryMsgs->PyPoseStamped_FromPoseStamped(pose);
//     // PyObject* pyGlobalPath = navMsgs->PyPath_FromPath(globalPath);
//     // PyTuple_SetItem(arguments, 0, pyOccupancyGrid);
//     // PyTuple_SetItem(arguments, 1, pyPose);
//     // PyTuple_SetItem(arguments, 2, pyGlobalPath);

//     // PyObject* func_getVelocity = PyWrapper::GetFunction("artificial_potential_field.artificial_potential_field", "getVelocity");
//     // PyObject* value = PyObject_CallObject(func_getVelocity, arguments);
//     // if (value == NULL) {
//     //     std::cout << "value == NULL" << std::endl;
//     // }
//   }
// }

int main()
{
    Py_Initialize();
    // ModuleImport();
    // GetFunction();
    GetFunctionRaw();
    Py_Finalize();

    //   dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
    // ComputeVelocity();
    //   ImportTest();
    // ThroughPut();

    return 0;
}