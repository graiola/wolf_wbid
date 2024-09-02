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


#ifndef __XBOTINTERFACE_UTILS_H__
#define __XBOTINTERFACE_UTILS_H__

#include <XBotInterface/RtLog.hpp>
#include <XBotInterface/TypedefAndEnums.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <streambuf>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <stdio.h>

namespace XBot {
namespace Utils {

/**
    * @brief Computes a matrix S such that S(a)b = a x b
    *
    * @param v A 3D vector
    * @return The skew symmetric matrix which represents a vector product by v
    */
inline Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& v){

    Eigen::Matrix3d S;

    S <<      0, -v.z(),  v.y(),
            v.z(),      0, -v.x(),
            -v.y(),  v.x(),      0;

    return S;
}


/**
 * @brief Computes an orientation error between two frames such that
 * an angular velocity K*e (K > 0) brings "actual" towards "ref"
 *
 * @param ref Reference orientation
 * @param actual Current orientation
 * @param error The orientation error between ref and actual
 * @return void
 */
inline void computeOrientationError(const Eigen::Matrix3d& ref,
                                    const Eigen::Matrix3d& actual,
                                    Eigen::Vector3d& error)
{

    Eigen::Quaterniond q(actual), q_d(ref);

    if(q.dot(q_d) < 0){
        q.x() *= -1;
        q.y() *= -1;
        q.z() *= -1;
        q.w() *= -1;
    }

    error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());

}





inline void FifthOrderTrajectory(const double init_time,
                                 const Eigen::VectorXd& _startPosture,
                                 const Eigen::VectorXd& _targetPosture,
                                 const double _max_vel,
                                 const double traj_time,
                                 Eigen::VectorXd& ref,
                                 Eigen::VectorXd& ref_dot,
                                 double& _duration_)
{ 
    double _t1;
    double duration_temp 	= 0.0;
    int _vec_size  	= _startPosture.size();
    // Find the trajectory duration
    _duration_ 		= 0.0;
    for (int i=0; i<_vec_size; i++)
    {
        _t1 		= _targetPosture(i) - _startPosture(i);
        duration_temp 	= 1.8750 * std::abs(_t1) / _max_vel;
        if (duration_temp > _duration_)
            _duration_ 	= duration_temp;
    }
    // Generate the trajectory
    double _time_ 	= traj_time - init_time;
    if (_time_>= 0 && _time_<= _duration_)
    {
        double tr 		= (_time_/_duration_);
        double tr2 		= tr * tr;
        double tr3 		= tr2 * tr;
        double s 		=  (6.0 * tr3 * tr2 		- 15.0 * tr3 * tr 	+ 10.0 * tr3);
        double sd 		= (30.0 * tr3 * tr/_duration_ 	- 60.0 * tr3/_duration_ + 30.0 * tr2/_duration_);
        //
        ref 			= _startPosture + (_targetPosture - _startPosture) * s;
        ref_dot 		= (_targetPosture - _startPosture) * sd;
    }
    else if (_time_<0) {
        ref 		= _startPosture;
        ref_dot 		= 0.0 * _startPosture;
    }
    else {
        ref 		= _targetPosture;
        ref_dot 		= 0.0 * _targetPosture;
    }
}





inline void ThirdOrderTrajectory(const double init_time,
                                 const double init_pos,
                                 const double final_pos,
                                 const double max_vel,
                                 const double traj_time,
                                 double& ref,
                                 double& ref_dot,
                                 double& duration)
{ 
    double time_t;
    double length;
    length = final_pos - init_pos;
    time_t = traj_time - init_time;
    duration    = 1.5 * length / max_vel;
    if (time_t>= 0 && time_t<= duration) {
        ref     = -2.0 * length * std::pow(time_t/duration,3) + 3.0 * length * std::pow(time_t/duration,2) + init_pos;
        ref_dot   = -6.0 * length * std::pow(time_t,2)/std::pow(duration,3) + 6.0 * length * time_t/std::pow(duration,2);
    }
    else if (time_t<0) {
        ref = init_pos;
        ref_dot = 0;
    }
    else {
        ref = final_pos;
        ref_dot = 0;
    }
} 


template <typename SignalType>
/**
 * @brief SecondOrderFilter implements a canonical continuous-time
 * second order filter with transfer function
 *                   1
 * P(s) =  -----------------------,  w = natural frequency, eps = damping ratio
 *         (s/w)^2 + 2*eps/w*s + 1
 *
 * and discretized according to a trapezoidal (aka Tustin) scheme. This yields
 * a difference equation of the following form:
 *
 *      a0*y + a1*yd + a2*ydd = u + b1*ud + b2*udd
 *
 * where yd = y(k-1), ydd = y(k-2) and so on (d = delayed).
 */
class SecondOrderFilter {

public:

    typedef std::shared_ptr<SecondOrderFilter<SignalType>> Ptr;

    SecondOrderFilter():
        _omega(1.0),
        _eps(0.8),
        _ts(0.01),
        _reset_has_been_called(false)
    {
        computeCoeff();
    }

    SecondOrderFilter(double omega, double eps, double ts, const SignalType& initial_state):
        _omega(omega),
        _eps(eps),
        _ts(ts),
        _reset_has_been_called(false)
    {
        computeCoeff();
        reset(initial_state);
    }

    void reset(const SignalType& initial_state){
        _reset_has_been_called = true;
        _u = initial_state;
        _y = initial_state;
        _yd = initial_state;
        _ydd = initial_state;
        _udd = initial_state;
        _ud = initial_state;
    }

    const SignalType& process(const SignalType& input){

        if(!_reset_has_been_called) reset(input*0);


        _ydd = _yd;
        _yd = _y;
        _udd = _ud;
        _ud = _u;


        _u = input;
        _y = 1.0/_a0 * ( _u + _b1*_ud + _b2*_udd - _a1*_yd - _a2*_ydd );

        return _y;
    }

    const SignalType& getOutput() const {
        return _y;
    }

    void setOmega(double omega){
        _omega = omega;
        computeCoeff();
    }

    double getOmega()
    {
        return _omega;
    }

    void setDamping(double eps){
        _eps = eps;
        computeCoeff();
    }

    double getDamping()
    {
        return _eps;
    }

    void setTimeStep(double ts){
        _ts = ts;
        computeCoeff();
    }

    double getTimeStep()
    {
        return _ts;
    }

private:

    void computeCoeff()
    {
        _b1 = 2.0;
        _b2 = 1.0;

        _a0 = 1.0 + 4.0*_eps/(_omega*_ts) + 4.0/std::pow(_omega*_ts, 2.0);
        _a1 = 2 - 8.0/std::pow(_omega*_ts, 2.0);
        _a2 = 1.0 + 4.0/std::pow(_omega*_ts, 2.0) - 4.0*_eps/(_omega*_ts);

    }

    double _omega;
    double _eps;
    double _ts;

    double _b1, _b2;
    double _a0, _a1, _a2;

    bool _reset_has_been_called;

    SignalType _y, _yd, _ydd, _u, _ud, _udd;

};

template<class SignalType>
class SecondOrderFilterArray
{
public:
    SecondOrderFilterArray(const int channels):
        _channels(channels)
    {
        SecondOrderFilter<SignalType> filter;
        SignalType tmp;
        for(unsigned int i = 0; i < _channels; ++i){
            _filters.push_back(filter);
            _output.push_back(tmp);
        }
    }

    const std::vector<SignalType>& process(const std::vector<SignalType>& input)
    {
        if(input.size() != _channels)
            throw std::runtime_error("input size != filters size");

        for(unsigned int i = 0; i < _channels; ++i){
            _filters[i].process(input[i]);
            _output[i] = _filters[i].getOutput();
        }
        return _output;
    }

    const std::vector<SignalType>& getOutput() const {
        return _output;
    }

    void reset(const SignalType& init)
    {
        for(unsigned int i = 0; i < _channels; ++i){
            _filters[i].reset(init);
            _output[i] = init;
        }
    }

    bool setTimeStep(const double time_step, const int channel)
    {
        if(channel >= _channels)
            return false;
        else
            _filters[channel].setTimeStep(time_step);
        return true;
    }

    double getTimeStep(const int channel)
    {
        if(channel >= _channels)
            throw std::runtime_error("channel out of channels range!");
        else
            return _filters[channel].getTimeStep();
    }

    bool setDamping(const double damping, const int channel)
    {
        if(channel >= _channels)
            return false;
        else
            _filters[channel].setDamping(damping);
        return true;
    }

    double getDamping(const int channel)
    {
        if(channel >= _channels)
            throw std::runtime_error("channel out of channels range!");
        else
            return _filters[channel].getDamping();
    }

    bool setOmega(const double omega, const int channel)
    {
        if(channel >= _channels)
            return false;
        else
            _filters[channel].setOmega(omega);
        return true;
    }

    double getOmega(const int channel)
    {
        if(channel >= _channels)
            throw std::runtime_error("channel out of channels range!");
        else
            return _filters[channel].getOmega();
    }

    int getNumberOfChannels()
    {
        return _channels;
    }

private:
    std::vector<SecondOrderFilter<SignalType>> _filters;
    std::vector<SignalType> _output;
    int _channels;

};

/**
 * @brief exec runs a command inside the shell, and returns the output
 */
inline std::string exec(const char* cmd)
{
    FILE* pipe = popen(cmd, "r");

    if (!pipe)
    {
        throw std::runtime_error("popen() failed");
    }

    std::string result;

    try
    {
        char buffer[128];

        while(fgets(buffer, sizeof buffer, pipe) != NULL)
        {
            result += buffer;
        }
    }
    catch(...)
    {
        pclose(pipe);
        throw;
    }

    int retcode = pclose(pipe);

    if(retcode != 0)
    {
        throw std::runtime_error("child process returned " + std::to_string(retcode));
    }

    return result;
}

/**
 * @brief Computes the absolute path corresponging to a given path relative to the $XBOT_ROOT
 * environment variable.
 */
inline std::string computeAbsolutePath(const std::string& input_path)
{
    // if not an absolute path
    if(input_path == "" || !(input_path.at(0) == '/'))
    {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("XBOT_ROOT");

        // check the env, otherwise error
        if(env_p)
        {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += "/";
            current_path += input_path;
            return current_path;
        }
        else
        {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return "";
        }
    }
    
    // already an absolute path
    return input_path;
}

/**
 * @brief computeAbsolutePathShell expands the provided path by using
 * the system shell before pre-pending the XBOT_ROOT value
 */
inline std::string computeAbsolutePathShell(std::string input_path, std::string wd = ".")
{
    // define shell command
    std::string cmd = "set -eu; cd " + wd + "; /bin/echo " + input_path;

    std::string cmd_output;

    try
    {
        cmd_output = XBot::Utils::exec(cmd.c_str());
    }
    catch(std::exception& e)
    {
        Logger::error("command '%s' failed: %s \n",
                      cmd.c_str(), e.what());
        return "";
    }

    // check for error from echo
    if(cmd_output.empty())
    {
        return cmd_output;
    }

    // strip trailing \n
    cmd_output = cmd_output.substr(0, cmd_output.size() - 1);

    // further process via computeAbsolutePath()
    return computeAbsolutePath(cmd_output);
}

/**
 * @brief getter for the default XBot config file set by the env variable $XBOT_CONFIG
 */
inline std::string getXBotConfig()
{
    const char* env_p = std::getenv("XBOT_CONFIG");
    // check the env, otherwise error
    if(env_p) {
        std::string xbot_config(env_p);
        Logger::info(Logger::Severity::DEBUG) << "$XBOT_CONFIG" << " -> " << xbot_config << Logger::endl();
        std::ifstream fcp(xbot_config, std::ifstream::in);
        char xbot_config_path[300];
        fcp.getline (xbot_config_path, 300);

        Logger::info(Logger::Severity::DEBUG) << __func__ << " -> " << std::string(xbot_config_path) << Logger::endl();
        return std::string(xbot_config_path);
    }
    else {
        Logger::warning() << "WARNING in " << __func__ << " : XBOT_CONFIG env variable not set." << Logger::endl();
        return "";
    }
}

inline Eigen::Matrix6d GetAdjointFromRotation(const Eigen::Matrix3d& R){
    
    Eigen::Matrix6d I;
    I.setZero();
    
    I.block<3,3>(0,0) = R;
    I.block<3,3>(3,3) = R;
    
    return I;
    
}


template <typename T>   
class LimitedDeque {
    
public:
    
    LimitedDeque(int N = 0);
    
    void push_back(const T& elem);
    void push_back();
    bool pop_back();
    
    T& back();
    const T& back() const;
    
    int size() const;
    int capacity() const { return N; }
    
    bool is_full() const;
    bool is_empty() const;
    
    void reset();
    
private:
    
    const int N;
    
    int decrement_mod(int idx);
    
    int _oldest;
    int _newest;
    
    std::vector<T> _buffer;

};

template <typename T>
inline LimitedDeque<T>::LimitedDeque(int _N):
    N(_N)
{
    reset();
    
    _buffer.resize(N);
}

template <typename T>
inline const T& LimitedDeque<T>::back() const
{
    if(is_empty()){
        throw std::out_of_range("back() called on a empty deque");
    }
    else{
        int back_idx = decrement_mod(_newest);
        return _buffer.at(back_idx);
    }
    
}

template <typename T>
T& LimitedDeque<T>::back()
{
    if(is_empty()){
        throw std::out_of_range("back() called on a empty deque");
    }
    else{
        int back_idx = decrement_mod(_newest);
        return _buffer.at(back_idx);
    }
}


template <typename T>
inline bool LimitedDeque<T>::pop_back()
{
    if(is_empty()){
        return false;
    }
    else{
        _newest = decrement_mod(_newest);
        if( _newest == _oldest )
        {
            reset();
        }
        
        return true;
    }
}

template <typename T>
inline void LimitedDeque<T>::push_back(const T& elem)
{
    push_back();
    back() = elem;
}

template <typename T>
inline void LimitedDeque<T>::push_back()
{
    if(is_full()){
        
        
        _oldest = ( _oldest + 1 ) % N;
        _newest = _oldest;
        
    }
    else{
        
        if(is_empty()){
            _oldest = 0;
        }
        
        _newest = ( _newest + 1 ) % N;
        
    }
}  


template <typename T>
inline int LimitedDeque<T>::size() const
{
    if ( is_full() ) {
        return N;
    }
    else if ( is_empty() ) {
        return 0;
    }
    else if ( _newest > _oldest ) {
        return _newest - _oldest;
    }
    else {
        return ( N - _oldest + _newest );
    }
}

template <typename T>
inline bool LimitedDeque<T>::is_full() const
{
    return _oldest == _newest;
}

template <typename T>
inline bool LimitedDeque<T>::is_empty() const
{
    return !is_full() && _newest == 0 && _oldest == -1;
}

template <typename T>
inline int LimitedDeque<T>::decrement_mod(int idx)
{
    idx--;
    return idx >= 0 ? idx : (idx + N);
}

template <typename T>
inline void LimitedDeque<T>::reset()
{
    _oldest = -1;
    _newest = 0;
}


inline void FifthOrderPlanning(double x0, double dx0, double ddx0, 
                               double goal, double start_time, double end_time,
                               double time, double& x, double& dx, double& ddx
                               )
{
    Eigen::Matrix6d A;
    A << 1.0000,         0,         0,         0,         0,         0,
            0,    1.0000,         0,         0,         0,         0,
            0,         0,    0.5000,         0,         0,         0,
            -10.0000,   -6.0000,   -1.5000,   10.0000,   -4.0000,    0.5000,
            15.0000,    8.0000,    1.5000,  -15.0000,    7.0000,   -1.0000,
            -6.0000,   -3.0000,   -0.5000,    6.0000,   -3.0000,    0.5000;
    
    double alpha = (end_time-start_time);
    alpha = std::max(1e-6, alpha);
    double tau = (time - start_time)/alpha; // d/dt = d/d(alpha*tau)
    tau = std::max(0.0, tau);
    tau = std::min(tau, 1.0);

    Eigen::Vector6d b;
    b << x0, dx0*alpha, ddx0*std::pow(alpha,2.0), goal, 0.0, 0.0;
    
    Eigen::Vector6d coeffs = A*b;
    
    Eigen::Vector6d t_v, dt_v, ddt_v;
    for(int i = 0; i < 6; i++)
    {
        t_v(i) = std::pow(tau, i);
        dt_v(i) = i > 0 ? std::pow(tau, i-1)*i : 0;
        ddt_v(i) = i > 1 ? std::pow(tau, i-2)*i*(i-1) : 0;
        
    }
    
    x = coeffs.dot(t_v);
    dx = coeffs.dot(dt_v)/alpha;
    ddx = coeffs.dot(ddt_v)/(alpha*alpha);
}


inline std::string FindLib(std::string lib_name, std::string env)
{
    char * env_value_char = std::getenv(env.c_str());
    std::string env_value(env_value_char);
    
    Logger::info( Logger::Severity::DEBUG, "FindLib: env value is %s\n", env_value_char);
    
    std::vector<std::string> path_list;
    boost::split(path_list, env_value, [](char c){return c == ':';});
    
    std::sort( path_list.begin(), path_list.end() );
    path_list.erase( std::unique( path_list.begin(), path_list.end() ), path_list.end() );

    // NOTE add system folders
    path_list.push_back("/usr/lib");
    path_list.push_back("/usr/local/lib");

    Logger::info(Logger::Severity::DEBUG) << "Single paths are: \n";
    for(auto path : path_list)
    {
        Logger::info(Logger::Severity::DEBUG) << "---  " << path << "\n";
    }
    Logger::info(Logger::Severity::DEBUG) << Logger::endl();
    
    
    std::vector<boost::filesystem::path> found_items;
    
    
    for(auto path_string : path_list)
    {
        boost::filesystem::path current_path(path_string);
        if(!boost::filesystem::is_directory(current_path))
        {
            Logger::warning(Logger::Severity::DEBUG, "FindLib: Path \"%s\" is not a directory\n", path_string.c_str());
            continue;
        }
        
        boost::filesystem::directory_iterator it_begin(current_path);
        boost::filesystem::directory_iterator it_end;
        
        for (auto it = it_begin; it != it_end; it++){
            if(it->path().filename() == lib_name)
            {
                found_items.push_back(it->path());
            }
        }
    }
    
    
    
    if(found_items.size() == 0)
    {
        Logger::error("File %s NOT found\n", lib_name.c_str());
        return std::string();
    }
    else if(found_items.size() == 1)
    {
        Logger::success(Logger::Severity::DEBUG) << "Found item " << found_items[0] <<  Logger::endl();
    }
    else
    {
        Logger::warning(Logger::Severity::DEBUG, "Found more than one item:\n");
        for(auto& item : found_items)
        {
            Logger::warning(Logger::Severity::DEBUG) << "----- " << item << Logger::endl();
        }
    }
    
    return found_items[0].string();
    
}


inline bool ReadFile(std::string path_to_file, std::string& file)
{
    std::ifstream t(path_to_file);
    
    if(t.fail())
    {
        return false;
    }
    
    file = std::string((std::istreambuf_iterator<char>(t)),
                       std::istreambuf_iterator<char>());
    
    return true;
    
}

inline std::string GetTimeAsString()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    memset(buffer, 0, 80*sizeof(buffer[0]));

    std::time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 80, "%Y_%m_%d__%H_%M_%S", timeinfo);
    
    return std::string(buffer);
}

}

}

#endif
