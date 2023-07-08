#include "nav2_pyif/python_interface.hpp"

#define assertm(exp, msg) assert(((void)msg, exp))

pyif::PyMap::PyModule::PyModule(std::string module_name) {
    this->py_module_ = PyImport_ImportModule(module_name.c_str());
}
                
pyif::PyMap::PyModule::~PyModule() {
    Py_XDECREF(py_module_);
    for (auto function : functions_) {
        Py_XDECREF(function.second);
    }
}

PyObject *pyif::PyMap::PyModule::GetFunction(std::string function_name) {
    auto function = functions_.find(function_name);

    if (function == functions_.end()) {
        function = functions_.insert({function_name, PyObject_GetAttrString(py_module_, function_name.c_str())}).first;
    }

    return function->second;
}

void pyif::PyMap::Init(std::vector<std::pair<std::string, std::string>> modules_functions) {
    // Load the python library
    dlopen(PYTHON_LIB, RTLD_LAZY | RTLD_GLOBAL);

    Py_Initialize();

    for (auto module_function : modules_functions) {
        GetFunction(module_function.first, module_function.second);
    }
}

void pyif::PyMap::DeInit() {
    for (auto module : modules_) {
        module.second->pyif::PyMap::PyModule::~PyModule();
    }

    Py_Finalize();
}

std::shared_ptr<pyif::PyMap::PyModule> pyif::PyMap::GetModule(std::string module_name) {
    auto module = modules_.find(module_name);
    
    if (module == modules_.end()) {
        module = modules_.insert({module_name, std::shared_ptr<pyif::PyMap::PyModule>(new pyif::PyMap::PyModule(module_name))}).first;
    }

    return module->second;
}

PyObject* pyif::PyMap::GetFunction(std::string module_name, std::string function_name) {
    auto module = GetModule(module_name);

    assertm(module != NULL, "module is NULL");
    
    return module->GetFunction(function_name);
}