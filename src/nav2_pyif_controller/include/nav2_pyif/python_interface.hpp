#ifndef PYIF_HPP
#define PYIF_HPP

#include <unordered_map>
#include <iostream>
#include <memory>
#include <vector>
#include <dlfcn.h>

#include <Python.h>

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

namespace pyif {

class PyMap {
    private:
        class PyModule {
            public:
                PyModule(std::string module_name) {
                    this->py_module_ = PyImport_ImportModule(module_name.c_str());
                }
                
                ~PyModule() {
                    Py_XDECREF(py_module_);
                    for (auto function : functions_) {
                        Py_XDECREF(function.second);
                    }
                }

                PyObject* GetFunction(std::string function_name) {
                    auto function = functions_.find(function_name);

                    if (function == functions_.end()) {
                        function = functions_.insert({function_name, PyObject_GetAttrString(py_module_, function_name.c_str())}).first;
                    }

                    return function->second;
                }

            private:
                PyObject* py_module_ = NULL;
                std::unordered_map<std::string, PyObject*> functions_;
        };

    public:
        static void Init(std::vector<std::pair<std::string, std::string>> modules_functions) {

            // Load the python library
            dlopen(PYTHON_LIB, RTLD_LAZY | RTLD_GLOBAL);

            Py_Initialize();

            for (auto module_function : modules_functions) {
                GetFunction(module_function.first, module_function.second);
            }
        }

        static void DeInit() {
            for (auto module : modules_) {
                module.second->~PyModule();
            }

            Py_Finalize();
        }

        static std::shared_ptr<PyModule> GetModule(std::string module_name) {
            auto module = modules_.find(module_name);
            
            if (module == modules_.end()) {
                module = modules_.insert({module_name, std::shared_ptr<PyModule>(new PyModule(module_name))}).first;
            }

            return module->second;
        }

        static PyObject* GetFunction(std::string module_name, std::string function_name) {
            auto module = GetModule(module_name);

            if (module == NULL) {
                std::cout << "module " << module_name << " == NULL" << std::endl;
                return NULL;
            }
            else {
                return module->GetFunction(function_name);
            }
        }
    
    private:
        inline static std::unordered_map<std::string, std::shared_ptr<PyModule>> modules_;
};

class PyConvert {
    public:
        template <typename input_T, typename output_T> 
        constexpr output_T Convert(input_T input, PyObject* optional_input = NULL);

    private:
        template <typename T> 
        constexpr PyObject* MsgToPython(T src_msg, PyObject* src_object);

        template <typename T> 
        constexpr T MsgFromPython(PyObject* src_object);
};

class StdMsgs {
    public:
        static builtin_interfaces::msg::Time PyStamp_AsStamp(PyObject* py_stamp);

        static PyObject* PyStamp_FromStamp(const builtin_interfaces::msg::Time& cpp_stamp, PyObject* py_stamp);

        static std_msgs::msg::Header PyHeader_AsHeader(PyObject* py_header);

        static PyObject* PyHeader_FromHeader(const std_msgs::msg::Header& cpp_header, PyObject* py_header);
};

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

#endif // PYIF_HPP