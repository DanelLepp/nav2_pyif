#ifndef NAV2_PY_WRAPPER_HPP
#define NAV2_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

#include <unordered_map>
#include <iostream>
#include <memory>

// This class takes the usual 200ms to load the python function the first time but every time after that it takes under 1us (~300ns)
// This makes getting python functions almost 1e6 times faster 
class PyWrapper {
    private:
        PyObject* pyModule = NULL;
        std::unordered_map<std::string, PyObject*> functions;

    private:
        PyWrapper(std::string moduleName) {
            if (pyModule == NULL) {
                pyModule = PyImport_ImportModule(moduleName.c_str());
                if (pyModule == NULL) {
                    std::cout << "module " << moduleName << " == NULL" << std::endl;
                }
            }
        }

    public:
        ~PyWrapper() {
            std::cout << "~PyWrapper" << std::endl;
        }
         
        static PyObject* GetFunction(std::string moduleName, std::string functionName) {
            static std::unordered_map<std::string, std::unique_ptr<PyWrapper>> modules;
            auto module = modules.find(moduleName);
            
            if (module != modules.end()) {
                auto function = module->second->functions.find(functionName);

                if (function != module->second->functions.end()) {
                    if (function->second == NULL) {
                        std::cout << "module " << moduleName << " == NULL" << std::endl;
                        return NULL;
                    }
                    // PyObject * class_attr = PyObject_GetAttrString(function->second, "__name__");
                    // PyObject * module_attr = PyObject_GetAttrString(function->second, "__module__");
                    // std::cout << "module " << (char *)PyUnicode_1BYTE_DATA(module_attr) << " class " << (char *)PyUnicode_1BYTE_DATA(class_attr) << std::endl;
                    return function->second; 
                } 
                else {
                    if (module->second->pyModule == NULL) {
                        std::cout << "module " << moduleName << " == NULL" << std::endl;
                        return NULL;
                    }

                    PyObject* pyFunction = PyObject_GetAttrString(module->second->pyModule, functionName.c_str());
                    Py_XDECREF(module->second->pyModule);

                    if (pyFunction == NULL) {
                        std::cout << "function " << functionName << " == NULL" << std::endl;
                        return NULL;
                    }

                    module->second->functions.insert({functionName, pyFunction});
                    Py_XDECREF(pyFunction);

                    return GetFunction(moduleName, functionName);
                }

            } 
            else {
                modules.insert({static_cast<std::string>(moduleName), std::unique_ptr<PyWrapper>(new PyWrapper(moduleName))});
                return GetFunction(moduleName, functionName);
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

#endif // NAV2_PY_WRAPPER_HPP