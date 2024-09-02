/*
 * Copyright (C) 2018 IIT-ADVR
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

#ifndef __XBOT_CONFIG_OPTIONS_H__
#define __XBOT_CONFIG_OPTIONS_H__


#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <boost/any.hpp>
#include <yaml-cpp/yaml.h>
#include <XBotInterface/Logger.hpp>
#include <XBotInterface/Utils.h>

namespace XBot 
{
    
    class ConfigOptions
    {
    public:
        
        static ConfigOptions FromConfigFile(std::string path_to_config_file);
        
        const std::string& get_urdf() const;
        const std::string& get_srdf() const;
        const std::string& get_joint_id_map() const;

        /**
         * @brief get_urdf_path retrieve the path to the urdf passed
         * @param urdf_path path to the urdf
         * @return false if the class was initialized using a urdf instead
         */
        bool get_urdf_path(std::string& urdf_path);

        /**
         * @brief get_srdf_path retrieve the path to the srdf passed
         * @param srdf_path path to the srdf
         * @return false if the class was initialized using a srdf instead
         */
        bool get_srdf_path(std::string& srdf_path);
        
        bool has_config() const;
        const YAML::Node& get_config() const;
        std::string get_path_to_config() const;
        
        template <typename ParameterType>
        bool get_parameter(std::string key, ParameterType& value) const;
        
        template <typename ParameterType>
        bool set_parameter(std::string key, const ParameterType& value);
        
        bool is_same_robot(const ConfigOptions& other) const;
        
        bool set_urdf_path(std::string path_to_urdf);
        bool set_srdf_path(std::string path_to_srdf);
        bool set_jidmap_path(std::string path_to_jidmap);
        
        bool set_urdf(std::string urdf);
        bool set_srdf(std::string srdf);
        bool set_jidmap(std::string jidmap);

        bool generate_jidmap();
        
    protected:
        
        std::string _urdf, _srdf, _jidmap;
        std::string _urdf_path, _srdf_path;
        YAML::Node _config;
        std::string _path_to_cfg;
        
    private:
        
        
        std::map<std::string, boost::any> _any_values;
        
    };
    
    
    
}

template <typename ParameterType>
inline bool XBot::ConfigOptions::get_parameter(std::string key, ParameterType& value) const
{
    if(_any_values.count(key) == 0)
    {
        return false;
    }
    
    try
    {
        value = boost::any_cast<ParameterType>(_any_values.at(key));
    }
    catch(std::exception& e)
    {
        Logger::error("Unable to convert parameter %s to the desired type: %s", key.c_str(), e.what());
        return false;
    }
    
    return true;
    
}


template <typename ParameterType>
inline bool XBot::ConfigOptions::set_parameter(std::string key, const ParameterType& value)
{
    _any_values[key] = boost::any(value);
    return true;
}

#endif
