#include <XBotInterface/ConfigOptions.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom_advr/model.h>

#include <libgen.h>

const YAML::Node& XBot::ConfigOptions::get_config() const
{
    return _config;
}

const std::string& XBot::ConfigOptions::get_joint_id_map() const
{
    return _jidmap;
}

const std::string& XBot::ConfigOptions::get_srdf() const
{
    return _srdf;
}

const std::string& XBot::ConfigOptions::get_urdf() const
{
    return _urdf;
}

bool XBot::ConfigOptions::has_config() const
{
    return !_config.IsNull();
}

bool XBot::ConfigOptions::is_same_robot(const XBot::ConfigOptions& other) const
{
    return (_urdf == other._urdf) && (_srdf == other._srdf) && (_jidmap == other._jidmap);
}

std::string getDirName(std::string path)
{
    char * path_cstr = strdup(path.c_str());
    return dirname(path_cstr);
}


static std::string computeAbsolutePathShell(std::string input_path,
                                            std::string wd = ".")
{
    auto path = XBot::Utils::computeAbsolutePathShell(input_path, wd);
    if(path.empty())
    {
        throw std::runtime_error("shell failed interpreting '" +
                                 input_path + "'");
    }

    return path;
}


XBot::ConfigOptions XBot::ConfigOptions::FromConfigFile(std::string path_to_config_file)
{
    std::string urdf_path, srdf_path, jidmap_path;
    std::string framework, model_type;
    std::string ros_control_msg_type, ros_jointstate_msg_type;
    bool ros_publish_tf = true;
    bool is_model_floating_base = true;

    std::ifstream fin(path_to_config_file);
    if (fin.fail())
    {
        Logger::error() << "in " << __func__ << ": cannot open '" << path_to_config_file << "'" << Logger::endl();
        throw std::runtime_error("main config not available");
    }

    ConfigOptions opt;
    YAML::Node cfg = YAML::LoadFile(path_to_config_file);
    opt._path_to_cfg = path_to_config_file;

    YAML::Node xbi_node;
    if(cfg["XBotInterface"])
    {
        xbi_node = cfg["XBotInterface"];
    }
    else
    {
        Logger::error() << "in " << __func__ <<
                           " : YAML file '" << path_to_config_file <<
                           "' does not contain XBotInterface mandatory node"
                        << Logger::endl();
        throw std::runtime_error("main config is invalid");
    }

    // check the urdf_filename
    if(xbi_node["urdf_path"])
    {
        urdf_path = computeAbsolutePathShell(xbi_node["urdf_path"].as<std::string>(),
                getDirName(path_to_config_file));
    }
    else
    {
        Logger::error() << "in " << __func__ <<
                           ": XBotInterface node of '" << path_to_config_file <<
                           "' does not contain urdf_path mandatory node"
                        << Logger::endl();
        throw std::runtime_error("urdf not available");
    }

    // check the srdf_filename
    if(xbi_node["srdf_path"])
    {
        srdf_path = computeAbsolutePathShell(xbi_node["srdf_path"].as<std::string>(),
                getDirName(path_to_config_file));
    }
    else
    {
        Logger::error() << "in " << __func__ <<
                           ": XBotInterface node of '" << path_to_config_file <<
                           "' does not contain srdf_path mandatory node"
                        << Logger::endl();
        throw std::runtime_error("srdf not available");
    }

    // check joint_map_config
    if(xbi_node["joint_map_path"])
    {
        jidmap_path = computeAbsolutePathShell(xbi_node["joint_map_path"].as<std::string>(),
                getDirName(path_to_config_file));
    }

    opt._config = cfg;
    if(!opt.set_urdf_path(urdf_path))
    {
        throw std::runtime_error("'" + urdf_path + "' not found");
    }

    if(!opt.set_srdf_path(srdf_path))
    {
        throw std::runtime_error("'" + srdf_path + "' not found");
    }

    // if id map given, check path valid
    if(!jidmap_path.empty() &&
            !opt.set_jidmap_path(jidmap_path))
    {
        throw std::runtime_error("'" + jidmap_path + "' not found");
    }
    // generate id map if not given
    else if(jidmap_path.empty() && !opt.generate_jidmap())
    {
        throw std::runtime_error("could not generate joint id map");
    }


    /* RobotInterface parameters */
    YAML::Node robot_interface;
    if(cfg["RobotInterface"]) {
        robot_interface = cfg["RobotInterface"];

        if(robot_interface["framework_name"])
        {
            framework = robot_interface["framework_name"].as<std::string>();
            opt.set_parameter("framework", framework);
        }
    }

    /* ModelInterface parameters */
    YAML::Node model_interface;
    if(cfg["ModelInterface"]) {
        model_interface = cfg["ModelInterface"];

        if(model_interface["model_type"])
        {
            model_type = model_interface["model_type"].as<std::string>();
            opt.set_parameter("model_type", model_type);
        }

        if(model_interface["is_model_floating_base"])
        {
            is_model_floating_base = model_interface["is_model_floating_base"].as<bool>();
            opt.set_parameter("is_model_floating_base", is_model_floating_base);
        }
    }

    /* RobotInterfaceROS parameters */
    YAML::Node robot_interface_ros;
    if(cfg["RobotInterfaceROS"]) {
        robot_interface_ros = cfg["RobotInterfaceROS"];

        if(robot_interface_ros["control_message_type"])
        {
            ros_control_msg_type = robot_interface_ros["control_message_type"].as<std::string>();
            opt.set_parameter("control_message_type", ros_control_msg_type);
        }

        if(robot_interface_ros["jointstate_message_type"])
        {
            ros_jointstate_msg_type = robot_interface_ros["jointstate_message_type"].as<std::string>();
            opt.set_parameter("jointstate_message_type", ros_jointstate_msg_type);
        }

        if(robot_interface_ros["publish_tf"])
        {
            ros_publish_tf = robot_interface_ros["publish_tf"].as<bool>();
            opt.set_parameter("publish_tf", ros_publish_tf);
        }
    }


    return opt;

}

bool XBot::ConfigOptions::set_urdf_path(std::string path_to_urdf)
{
    _urdf_path = path_to_urdf;
    return XBot::Utils::ReadFile(path_to_urdf, _urdf);
}

bool XBot::ConfigOptions::set_srdf_path(std::string path_to_srdf)
{
    _srdf_path = path_to_srdf;
    return XBot::Utils::ReadFile(path_to_srdf, _srdf);
}

bool XBot::ConfigOptions::set_jidmap_path(std::string path_to_jidmap)
{
    return XBot::Utils::ReadFile(path_to_jidmap, _jidmap);
}

bool XBot::ConfigOptions::set_urdf(std::string urdf)
{
    if(!urdf.empty())
    {
        _urdf = urdf;
        return true;
    }
    else {
        return false;
    }
}

bool XBot::ConfigOptions::set_srdf(std::string srdf)
{
    if(!srdf.empty())
    {
        _srdf = srdf;
        return true;
    }
    else {
        return false;
    }
}

bool XBot::ConfigOptions::set_jidmap(std::string jidmap)
{
    if(!jidmap.empty())
    {
        _jidmap = jidmap;
        return true;
    }
    else {
        return false;
    }
}

std::string XBot::ConfigOptions::get_path_to_config() const
{
    return _path_to_cfg;
}

bool XBot::ConfigOptions::get_urdf_path(std::string& urdf_path)
{
    if(!_urdf_path.empty()){
        urdf_path = _urdf_path;
        return true;
    }
    return false;
}

bool XBot::ConfigOptions::get_srdf_path(std::string& srdf_path)
{
    if(!_srdf_path.empty()){
        srdf_path = _srdf_path;
        return true;
    }
    return false;
}

bool XBot::ConfigOptions::generate_jidmap()
{
    if(_urdf.empty())
    {
        XBot::Logger::error("in %s: urdf does not exists! \n", __func__);
        return false;
    }

    if(_srdf.empty())
    {
        XBot::Logger::error("in %s: srdf does not exists! \n", __func__);
        return false;
    }

    std::stringstream jidmap_stringstream;
    jidmap_stringstream<<"joint_map:"<<std::endl;

    auto urdf_model = urdf::parseURDF(_urdf);

    std::map<std::string, urdf::JointSharedPtr> joint_map = urdf_model->joints_;

    unsigned int i = 1;
    for(auto elem : joint_map)
    {
        urdf::JointSharedPtr joint = elem.second;
        if(joint->type == urdf::Joint::REVOLUTE ||
                joint->type == urdf::Joint::CONTINUOUS ||
                joint->type == urdf::Joint::PRISMATIC)
        {
            jidmap_stringstream<<"  "<<i<<": "<<joint->name<<std::endl;
            i++;
        }
    }

    srdf_advr::Model srdf_model;
    srdf_model.initString(*urdf_model, _srdf);

    std::vector<srdf_advr::Model::Group> srdf_groups= srdf_model.getGroups();
    for(unsigned int j = 0; j < srdf_groups.size(); ++j)
    {
        srdf_advr::Model::Group group = srdf_groups[j];
        if(group.name_ == "force_torque_sensors")
        {
            std::vector<std::string> group_joints =  group.joints_;
            for(unsigned int k = 0; k < group_joints.size(); ++k)
            {
                jidmap_stringstream<<"  "<<i<<": "<<group_joints[k]<<std::endl;
                i++;
            }
        }
        else if(group.name_ == "imu_sensors")
        {
            std::vector<std::string> group_links =  group.links_;
            for(unsigned int k = 0; k < group_links.size(); ++k)
            {
                jidmap_stringstream<<"  "<<i<<": "<<urdf_model->getLink(group_links[k])->parent_joint->name<<std::endl;
                i++;
            }
        }
    }


    _jidmap = jidmap_stringstream.str();

    XBot::Logger::info("Generated joint id map as:\n %s", _jidmap.c_str());

    return true;
}

