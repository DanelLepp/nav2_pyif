#ifndef PYIF_HPP
#define PYIF_HPP

#include <unordered_map>
#include <memory>
#include <vector>
#include <dlfcn.h>
#include <cassert>
#include <iostream>

#include "Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

namespace pyif {

class PyMap {
    private:
        class PyModule {
            public:
                PyModule(std::string module_name);
                
                ~PyModule();

                PyObject* GetFunction(std::string function_name);

            private:
                PyObject* py_module_ = NULL;
                std::unordered_map<std::string, PyObject*> functions_;
        };

    public:
        static void Init(std::vector<std::pair<std::string, std::string>> modules_functions);

        static void DeInit();

        static std::shared_ptr<PyModule> GetModule(std::string module_name);

        static PyObject* GetFunction(std::string module_name, std::string function_name);
    
    private:
        inline static std::unordered_map<std::string, std::shared_ptr<PyModule>> modules_;
};

} // namespace pyif

#endif // PYIF_HPP