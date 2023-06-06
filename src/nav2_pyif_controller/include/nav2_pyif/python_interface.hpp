#ifndef NAV2_PY_WRAPPER_HPP
#define NAV2_PY_WRAPPER_HPP

#include <Python.h>
#include <dlfcn.h>

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

#include <unordered_map>
#include <iostream>
#include <memory>
#include <vector>

namespace pyif {

class PyMap {
    private:
        class PyModule {
            public:
                PyObject* pyModule = NULL;
                std::unordered_map<std::string, PyObject*> functions;

                PyModule(std::string moduleName) {
                    this->pyModule = PyImport_ImportModule(moduleName.c_str());
                }
                
                ~PyModule() {
                    Py_XDECREF(pyModule);
                    for (auto function : functions) {
                        Py_XDECREF(function.second);
                    }
                }

                PyObject* GetFunction(std::string functionName) {
                    auto function = functions.find(functionName);

                    if (function == functions.end()) {
                        function = functions.insert({functionName, PyObject_GetAttrString(pyModule, functionName.c_str())}).first;
                    }

                    return function->second;
                }
        };

        static std::unordered_map<std::string, std::shared_ptr<PyModule>> modules;

    public:
        static void Init(std::vector<std::pair<std::string, std::string>> modulesFunctions) {

            // Load the python library
            dlopen(PYTHON_LIB, RTLD_LAZY | RTLD_GLOBAL);

            Py_Initialize();

            for (auto moduleFunction : modulesFunctions) {
                GetFunction(moduleFunction.first, moduleFunction.second);
            }
        }

        static void DeInit() {
            for (auto module : modules) {
                module.second->~PyModule();
            }

            Py_Finalize();
        }

        static std::shared_ptr<PyModule> GetModule(std::string moduleName) {
            auto module = modules.find(moduleName);
            
            if (module == modules.end()) {
                module = modules.insert({moduleName, std::shared_ptr<PyModule>(new PyModule(moduleName))}).first;
            }

            return module->second;
        }

        static PyObject* GetFunction(std::string moduleName, std::string functionName) {
            auto module = GetModule(moduleName);

            if (module == NULL) {
                std::cout << "module " << moduleName << " == NULL" << std::endl;
                return NULL;
            }
            else {
                return module->GetFunction(functionName);
            }
        }

    //     template <typename input_T, typename output_T> 
    //     constexpr output_T Convert(input_T input, PyObject* optionalInput = NULL) {
    //         if(std::is_same_v<input_T, PyObject*>) {
    //             return MsgFromPython(input);
    //         }
    //         else if(std::is_same_v<output_T, PyObject*>) {
    //             return MsgToPython(input, optionalInput);
    //         }
    //     }

    // private:
    //     template <typename T> 
    //     constexpr PyObject* MsgToPython(T srcMsg, PyObject* srcObject) {
    //         if (std::is_same_v<T, builtin_interfaces::msg::Time>) {
    //             return stdMsgs->PyStamp_FromStamp(srcMsg, srcObject);
    //         } else if(std::is_same_v<T, std_msgs::msg::Header>) {
    //             return stdMsgs->PyHeader_FromHeader(srcMsg, srcObject);
    //         }
    //     }

    //     template <typename T> 
    //     constexpr T MsgFromPython(PyObject* srcObject) {
    //         if(std::is_same_v<T, builtin_interfaces::msg::Time>) {
    //             return stdMsgs->PyStamp_AsStamp(srcObject);
    //         } else if(std::is_same_v<T, std_msgs::msg::Header>) {
    //             return stdMsgs->PyHeader_AsHeader(srcObject);
    //         }
    //     }
};

class MsgsBase {
    public:
        friend PyObject* PyMap::GetFunction(std::string moduleName, std::string functionName);
};

class StdMsgs : public MsgsBase {
    public:
        static builtin_interfaces::msg::Time PyStamp_AsStamp(PyObject* pyStamp);

        static PyObject* PyStamp_FromStamp(const builtin_interfaces::msg::Time& cppStamp, PyObject* pyStamp);

        static std_msgs::msg::Header PyHeader_AsHeader(PyObject* pyHeader);

        static PyObject* PyHeader_FromHeader(const std_msgs::msg::Header& cppHeader, PyObject* pyHeader);
};

class GeoMsgs : public MsgsBase {
    public:
        static geometry_msgs::msg::Vector3 PyVector3_AsVector3(PyObject* pyVector3);

        static PyObject* PyVector3_FromVector3(const geometry_msgs::msg::Vector3& cppVector3, PyObject* pyVector3);

        static geometry_msgs::msg::Point PyPoint_AsPoint(PyObject* pyPoint);

        static PyObject* PyPoint_FromPoint(const geometry_msgs::msg::Point& cppPoint, PyObject* pyPoint);

        static geometry_msgs::msg::Quaternion PyOrientation_AsOrientation(PyObject* pyOrientation);

        static PyObject* PyOrientation_FromOrientation(const geometry_msgs::msg::Quaternion& cppOrientation, PyObject* pyOrientation);

        static geometry_msgs::msg::Pose PyPose_AsPose(PyObject* pyPose);

        static PyObject* PyPose_FromPose(const geometry_msgs::msg::Pose& cppPose, PyObject* pyPose);

        static geometry_msgs::msg::PoseStamped PyPoseStamped_AsPoseStamped(PyObject* pyPoseStamped);

        static PyObject* PyPoseStamped_FromPoseStamped(const geometry_msgs::msg::PoseStamped& cppPoseStamped, PyObject* pyPoseStamped);

        static geometry_msgs::msg::Twist PyTwist_AsTwist(PyObject* pyTwist);

        static PyObject* PyTwist_FromTwist(const geometry_msgs::msg::Twist& cppTwist, PyObject* pyTwist);

        static geometry_msgs::msg::TwistStamped PyTwistStamped_AsTwistStamped(PyObject* pyTwistStamped);

        static PyObject* PyTwistStamped_FromTwistStamped(const geometry_msgs::msg::TwistStamped& cppTwistStamped, PyObject* pyTwistStamped);
};

class NavMsgs : public MsgsBase {
    public:
        static nav_msgs::msg::MapMetaData PyMapMetaData_AsMapMetaData(PyObject* pyMapMetaData);

        static PyObject* PyMapMetaData_FromMapMetaData(const nav_msgs::msg::MapMetaData& cppMapMetaData, PyObject* pyMapMetaData);

        static nav_msgs::msg::OccupancyGrid PyOccupancyGrid_AsOccupancyGrid(PyObject* pyOccupancyGrid);

        static PyObject* PyOccupancyGrid_FromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& cppOccupancyGrid, PyObject* pyOccupancyGrid);

        static nav_msgs::msg::Path PyPath_AsPath(PyObject* pyPath);

        static PyObject* PyPath_FromPath(const nav_msgs::msg::Path& cppPath, PyObject* pyPath);
};

}; // namespace pyif

std::unordered_map<std::string, std::shared_ptr<pyif::PyMap::PyModule>> pyif::PyMap::modules;

#endif // NAV2_PY_WRAPPER_HPP