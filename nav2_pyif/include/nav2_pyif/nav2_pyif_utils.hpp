#include "python_interface.hpp"
#include "geo_msgs.hpp"
#include "std_msgs.hpp"
#include "nav_msgs.hpp"

namespace pyif {

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

} // namespace pyif