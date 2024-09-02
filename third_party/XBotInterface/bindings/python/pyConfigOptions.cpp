#include <XBotInterface/ConfigOptions.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace XBot;

auto construct_options_from_xbot_config = []()
{
    return ConfigOptions::FromConfigFile(Utils::getXBotConfig());
};

auto set_string_parameter = [](ConfigOptions& opt, std::string name, std::string value)
{
    opt.set_parameter(name, value);
};

auto set_bool_parameter = [](ConfigOptions& opt, std::string name, bool value)
{
    opt.set_parameter(name, value);
};

PYBIND11_MODULE(config_options, m) {
    
    py::class_<ConfigOptions>(m, "ConfigOptions")
            .def(py::init<>())
            .def_static("FromConfigFile", construct_options_from_xbot_config)
            .def("get_urdf", &ConfigOptions::get_urdf)
            .def("get_srdf", &ConfigOptions::get_srdf)
            .def("set_urdf", &ConfigOptions::set_urdf)
            .def("set_srdf", &ConfigOptions::set_srdf)
            .def("set_string_parameter", set_string_parameter)
            .def("set_bool_parameter", set_bool_parameter)
            .def("get_joint_id_map", &ConfigOptions::get_joint_id_map)
            .def("generate_jidmap", &ConfigOptions::generate_jidmap);
    
}



