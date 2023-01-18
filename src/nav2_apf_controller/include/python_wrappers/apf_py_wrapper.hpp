#ifndef APF_PY_WRAPPER_HPP
#define APF_PY_WRAPPER_HPP

#include "python3.10/Python.h"

#define Py_DEBUG
#define PY_SSIZE_T_CLEAN

#include "geometry_msgs.hpp"
#include "nav_msgs.hpp"

class ArtificialPotentialField {
    private:
        PyObject* module_Apf = NULL;
        PyObject* func_getVelocity = NULL;
        PyObject* arguments = NULL;
        PyObject* value = NULL;
        GeometryMsgs* geometryMsgs;
        NavMsgs* navMsgs;

        static ArtificialPotentialField* pinstance_;
        static std::mutex mutex_;

    protected:
        ArtificialPotentialField() {
            if (module_Apf == NULL) {
                module_Apf = PyImport_ImportModule("artificial_potential_field.artificial_potential_field");
                if (module_Apf == NULL) {
                    std::cout << "module_Apf == NULL" << std::endl;
                }
                else if (func_getVelocity == NULL) {
                    func_getVelocity = PyObject_GetAttrString(module_Apf, "getVelocity");
                    if (func_getVelocity == NULL) {
                        std::cout << "getVelocity == NULL" << std::endl;
                    }
                }
            }

            if (arguments == NULL) {
                arguments = PyTuple_New(3);
                if (arguments == NULL) {
                    std::cout << "arguments == NULL" << std::endl;
                }
            }

            geometryMsgs = GeometryMsgs::GetInstance();
            navMsgs = NavMsgs::GetInstance();
        }

        ~ArtificialPotentialField() {
            Py_XDECREF(module_Apf);
            Py_XDECREF(func_getVelocity);
            Py_XDECREF(arguments);
        }

    public:
        ArtificialPotentialField(ArtificialPotentialField& other) = delete;
        void operator=(const ArtificialPotentialField&) = delete;

        static ArtificialPotentialField* GetInstance();

        geometry_msgs::msg::TwistStamped getVelocity(
            const nav_msgs::msg::OccupancyGrid& occupancyGrid,
            const geometry_msgs::msg::PoseStamped& pose,
            const nav_msgs::msg::Path& globalPath) 
        {
            std::lock_guard<std::mutex> lock(mutex_);
            PyTuple_SetItem(arguments, 0, navMsgs->PyOccupancyGrid_FromOccupancyGrid(occupancyGrid));
            PyTuple_SetItem(arguments, 1, geometryMsgs->PyPoseStamped_FromPoseStamped(pose));
            PyTuple_SetItem(arguments, 2, navMsgs->PyPath_FromPath(globalPath));

            value = PyObject_CallObject(func_getVelocity, arguments);
            
            return geometryMsgs->PyTwistStamped_AsTwistStamped(value);
        }
};

ArtificialPotentialField* ArtificialPotentialField::pinstance_{nullptr};
std::mutex ArtificialPotentialField::mutex_;

ArtificialPotentialField* ArtificialPotentialField::GetInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr) {
        pinstance_ = new ArtificialPotentialField();
    }
    return pinstance_;
}

#endif // APF_PY_WRAPPER_HPP