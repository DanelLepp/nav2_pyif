#ifndef NAV2_PY_WRAPPER_HPP
#define NAV2_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

#include <unordered_map>
#include <iostream>
#include <memory>
#include <vector>

// This class takes the usual 200ms to load the python function the first time but every time after that it takes under 1us (~300ns)
// This makes getting python functions almost 1e6 times faster 
class PyWrapper {
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

        // static PyObject* GetFunction(std::string moduleName, std::string functionName) {
        //     static std::unordered_map<std::string, std::unique_ptr<PyWrapper>> modules;
        //     auto module = modules.find(moduleName);
            
        //     if (module != modules.end()) {
        //         auto function = module->second->functions.find(functionName);

        //         if (function != module->second->functions.end()) {
        //             if (function->second == NULL) {
        //                 std::cout << "module " << moduleName << " == NULL" << std::endl;
        //                 return NULL;
        //             }
        //             // PyObject * class_attr = PyObject_GetAttrString(function->second, "__name__");
        //             // PyObject * module_attr = PyObject_GetAttrString(function->second, "__module__");
        //             // std::cout << "module " << (char *)PyUnicode_1BYTE_DATA(module_attr) << " class " << (char *)PyUnicode_1BYTE_DATA(class_attr) << std::endl;
        //             return function->second; 
        //         } 
        //         else {
        //             if (module->second->pyModule == NULL) {
        //                 std::cout << "module " << moduleName << " == NULL" << std::endl;
        //                 return NULL;
        //             }

        //             PyObject* pyFunction = PyObject_GetAttrString(module->second->pyModule, functionName.c_str());
        //             Py_XDECREF(module->second->pyModule);

        //             if (pyFunction == NULL) {
        //                 std::cout << "function " << functionName << " == NULL" << std::endl;
        //                 return NULL;
        //             }

        //             module->second->functions.insert({functionName, pyFunction});
        //             Py_XDECREF(pyFunction);

        //             return GetFunction(moduleName, functionName);
        //         }

        //     } 
        //     else {
        //         modules.insert({static_cast<std::string>(moduleName), std::unique_ptr<PyWrapper>(new PyWrapper(moduleName))});
        //         return GetFunction(moduleName, functionName);
        //     }
        // }

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

std::unordered_map<std::string, std::shared_ptr<PyWrapper::PyModule>> PyWrapper::modules;
#endif // NAV2_PY_WRAPPER_HPP