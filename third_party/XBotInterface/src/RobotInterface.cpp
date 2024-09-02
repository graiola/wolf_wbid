/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <dlfcn.h>
#include <XBotInterface/SoLib.h>
#include <XBotInterface/RtLog.hpp>
using XBot::Logger;

// NOTE Static members need to be defined in the cpp
std::map<std::string, XBot::RobotInterface::Ptr> XBot::RobotInterface::_instance_ptr_map;


XBot::RobotInterface::RobotInterface()
{
}

void XBot::RobotInterface::clearRobotMap(void)
{
    if(!_instance_ptr_map.empty())
        _instance_ptr_map.clear();
}



XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const ConfigOptions &config,
        const std::string &robot_name)
{

    std::string __robot_name;

    /* If robot_name is null, retrieve it from Urdf */
    if (robot_name == "") {
        auto urdfdom = urdf::parseURDF(config.get_urdf());
        if (!urdfdom) {
            throw std::runtime_error("unable to parse URDF");
        }
        __robot_name = urdfdom->name_;
    } else {
        __robot_name = robot_name;
    }



    /* Robots are managed as robot-wise singletons */
    if (_instance_ptr_map.count(__robot_name)) {
        if (!_instance_ptr_map.at(__robot_name)->getConfigOptions().is_same_robot(config)) {

            /* Same robot name AND different config file -> fatal error */
            throw std::runtime_error("Unmatching config files for requested robot!");
        } else {
            return _instance_ptr_map.at(__robot_name);
        }
    }

    /* Check that framework field is filled */
    std::string framework;
    if (!config.get_parameter("framework", framework)) {
        throw std::invalid_argument("Provided configuration object must contain a std::string-typed \"framework\" field");
    }

    /* Obtain full path to shared lib */
    std::string path_to_shared_lib = XBot::Utils::FindLib("libRobotInterface" + framework + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "") {
        throw std::runtime_error("libRobotInterface" + framework + ".so path must be listed inside LD_LIBRARY_PATH");
    }

    // loading the requested model interface internal to the robot
    auto instance_ptr  = SoLib::getFactory<XBot::RobotInterface>(path_to_shared_lib, "RobotInterface");
    if (instance_ptr) {
        instance_ptr->_model = XBot::ModelInterface::getModel(config);
        instance_ptr->init(config);
        _instance_ptr_map[__robot_name] = instance_ptr;
    }

    return _instance_ptr_map.at(__robot_name);

}

XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const std::string &path_to_cfg,
        const std::string &robot_name,
        AnyMapConstPtr any_map,
        const std::string &framework)
{

    std::string abs_path_to_cfg = XBot::Utils::computeAbsolutePath(path_to_cfg);
    auto cfg =  ConfigOptions::FromConfigFile(abs_path_to_cfg);

    if (framework != "") {
        cfg.set_parameter<std::string>("framework", framework);
    }

    return getRobot(cfg, robot_name);

}

XBot::ModelInterface &XBot::RobotInterface::model()
{
    return *_model;
}


bool XBot::RobotInterface::sense(bool sync_model)
{
    bool sense_ok = sense_internal();
    bool sense_hands_ok = sense_hands();
    bool sensors_ok = read_sensors();

    _ts_rx = getTime();

    if (sync_model) {
        return (_model->syncFrom(*this)) && sense_ok && sense_hands_ok && sensors_ok;
    }
    return sense_ok && sense_hands_ok && sensors_ok;
}


bool XBot::RobotInterface::move()
{
    _ts_tx = getTime();

    bool move_ok = move_internal();
    bool move_hands_ok = move_hands();

    return move_ok && move_hands_ok;
}

bool XBot::RobotInterface::init_internal(const ConfigOptions &config)
{
    // Fill _robot_chain_map with shallow copies of chains in _chain_map

    _robot_chain_map.clear();

    for (const auto & c : _chain_map) {

        RobotChain::Ptr robot_chain(new RobotChain());
        robot_chain->shallowCopy(*c.second);

        _robot_chain_map[c.first] = robot_chain;

    }

    // Call virtual init_robot
    bool success = init_robot(config);

    _ordered_chain_names.clear();
    for (const std::string & s : model().getModelOrderedChainName()) {
        if (s == "virtual_chain") {}
        else {
            _ordered_chain_names.push_back(s);
        }
    }

    // Since fixed controlled joints do not appear in the model, some chains may
    // be missing. So we add them!
    for (const auto & pair : _chain_map) {
        auto it = std::find(_ordered_chain_names.begin(), _ordered_chain_names.end(), pair.first);
        if (it == _ordered_chain_names.end()) {
            _ordered_chain_names.push_back(pair.first);
        }
    }

    return success;
}

bool XBot::RobotInterface::post_init()
{
    sense();

    Eigen::VectorXd tmp;
    getMotorPosition(tmp);
    setPositionReference(tmp);

    return true;
}



XBot::RobotChain &XBot::RobotInterface::arm(int arm_id)
{
    if (_XBotModel.get_arms_chain().size() > arm_id) {
        const std::string &requested_arm_name = _XBotModel.get_arms_chain().at(arm_id);
        return *_robot_chain_map.at(requested_arm_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a arms with id " << arm_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::RobotChain &XBot::RobotInterface::leg(int leg_id)
{
    if (_XBotModel.get_legs_chain().size() > leg_id) {
        const std::string &requested_leg_name = _XBotModel.get_legs_chain().at(leg_id);
        return *_robot_chain_map.at(requested_leg_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a legs with id " << leg_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::RobotChain &XBot::RobotInterface::operator()(const std::string &chain_name)
{
    if (_robot_chain_map.count(chain_name)) {
        return *_robot_chain_map.at(chain_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a chain with name " << chain_name << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::RobotChain &XBot::RobotInterface::chain(const std::string &chain_name)
{
    return operator()(chain_name);
}

const XBot::RobotChain &XBot::RobotInterface::arm(int arm_id) const
{
    if (_XBotModel.get_arms_chain().size() > arm_id) {
        const std::string &requested_arm_name = _XBotModel.get_arms_chain().at(arm_id);
        return *_robot_chain_map.at(requested_arm_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a arms with id " << arm_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

const XBot::RobotChain &XBot::RobotInterface::chain(const std::string &chain_name) const
{
    return operator()(chain_name);
}

const XBot::RobotChain &XBot::RobotInterface::leg(int leg_id) const
{
    if (_XBotModel.get_legs_chain().size() > leg_id) {
        const std::string &requested_leg_name = _XBotModel.get_legs_chain().at(leg_id);
        return *_robot_chain_map.at(requested_leg_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a legs with id " << leg_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

const XBot::RobotChain &XBot::RobotInterface::operator()(const std::string &chain_name) const
{
    if (_robot_chain_map.count(chain_name)) {
        return *_robot_chain_map.at(chain_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a chain with name " << chain_name << " that does not exists!!" << std::endl;
    return _dummy_chain;
}


double XBot::RobotInterface::getTimestampRx() const
{
    return _ts_rx;
}

double XBot::RobotInterface::getTimestampTx() const
{
    return _ts_tx;
}

bool XBot::RobotInterface::set_control_mode_internal(int joint_id, const XBot::ControlMode &control_mode)
{
    return false;
}


void XBot::RobotInterface::getControlMode(std::map< int, XBot::ControlMode > &control_mode) const
{
    ControlMode ctrl;
    for (int i = 0; i < getJointNum(); i++) {
        _joint_vector[i]->getControlMode(ctrl);
        control_mode[_joint_vector[i]->getJointId()] = ctrl;
    }
}

void XBot::RobotInterface::getControlMode(std::map< std::string, XBot::ControlMode > &control_mode) const
{
    ControlMode ctrl;
    for (int i = 0; i < getJointNum(); i++) {
        _joint_vector[i]->getControlMode(ctrl);
        control_mode[_joint_vector[i]->getJointName()] = ctrl;
    }
}



bool XBot::RobotInterface::setControlMode(const std::map< int, XBot::ControlMode > &control_mode)
{
    bool success = true;

    for (const auto & pair : control_mode) {


        if (hasJoint(pair.first)) {

            bool set_internal_success = set_control_mode_internal(pair.first, pair.second);

            if (set_internal_success) {
                getJointByIdInternal(pair.first)->setControlMode(pair.second);
//                 XBot::Logger::info(XBot::Logger::Severity::LOW) << "Joint " << getJointByIdInternal(pair.first)->getJointName() <<
//                           " - with id : " << pair.first << " - control mode changed to " << pair.second.getName() << XBot::Logger::endl();
            } else {
                XBot::Logger::warning(XBot::Logger::Severity::LOW) << "Joint " << getJointByIdInternal(pair.first)->getJointName() <<
                          " - with id : " << pair.first << " - CANNOT change control mode to " << pair.second.getName() << XBot::Logger::endl();
            }

            success = success && set_internal_success;
        }

    }

    return success;
}

bool XBot::RobotInterface::setControlMode(const std::map< std::string, XBot::ControlMode > &control_mode)
{
    bool success = true;

    for (const auto & pair : control_mode) {

        if (hasJoint(pair.first)) {

            bool set_internal_success = set_control_mode_internal(getJointByNameInternal(pair.first)->getJointId(), pair.second);

            if (set_internal_success) {
                getJointByNameInternal(pair.first)->setControlMode(pair.second);
//                 Logger::info() << "Joint " << pair.first << " - with id : "
//                                << getJointByNameInternal(pair.first)->getJointId()
//                                << " - control mode changed to " << pair.second.getName() << Logger::endl();
            } else {
                Logger::warning() << "Joint " << pair.first  << " - with id : "
                                  << getJointByNameInternal(pair.first)->getJointId() << " - CANNOT change control mode to "
                                  << pair.second.getName() << Logger::endl();
            }

            success = set_internal_success && success;
        }

    }

    return success;
}

bool XBot::RobotInterface::setControlMode(const XBot::ControlMode &control_mode)
{
    bool success = true;

    for (int i = 0; i < getJointNum(); i++) {

        bool set_internal_success = success = set_control_mode_internal(_joint_vector[i]->getJointId(), control_mode);

        if (set_internal_success) {
            _joint_vector[i]->setControlMode(control_mode);
//             Logger::info() << "Joint " << _joint_vector[i]->getJointName() << " - with id : "
//                            << _joint_vector[i]->getJointId()
//                            << " - control mode changed to " << control_mode.getName() << Logger::endl();
        } else {
            Logger::warning() << "Joint " << _joint_vector[i]->getJointName()  << " - with id : "
                              << _joint_vector[i]->getJointId() << " - CANNOT change control mode to "
                              << control_mode.getName() << Logger::endl();
        }

        success = set_internal_success && success;
    }

    return success;
}



bool XBot::RobotInterface::setControlMode(const std::string &chain_name, const XBot::ControlMode &control_mode)
{
    auto it = _chain_map.find(chain_name);

    if (it == _chain_map.end()) {
        Logger::error() << "ERROR in " << __func__ << "! Chain " << chain_name << " is NOT defined!" << Logger::endl();
        return false;
    }

    bool success = true;

    for (int i = 0; i < it->second->getJointNum(); i++) {

        bool set_internal_success = set_control_mode_internal(it->second->getJoint(i)->getJointId(), control_mode);

        if (set_internal_success) {
            it->second->getJointInternal(i)->setControlMode(control_mode);
//             Logger::info() << "Joint " << (it->second->getJoint(i)->getJointName()) << " - with id : "
//                            << (it->second->getJoint(i)->getJointId())
//                            << " - control mode changed to " << control_mode.getName() << Logger::endl();
        } else {
            Logger::warning() << "Joint " << (it->second->getJoint(i)->getJointName())  << " - with id : "
                              << (it->second->getJoint(i)->getJointId()) << " - CANNOT change control mode to "
                              << control_mode.getName() << Logger::endl();
        }

        success = success && set_internal_success;
    }

    return success;
}










