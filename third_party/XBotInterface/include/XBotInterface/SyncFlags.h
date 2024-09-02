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

#ifndef __XBOT_SYNCFLAGS__
#define __XBOT_SYNCFLAGS__

#include <XBotInterface/TypedefAndEnums.h>

namespace XBot {

    inline void parseFlags(bool& pos,
                           bool& vel,
                           bool& acc,
                           bool& eff,
                           bool& k,
                           bool& d,
                           bool& ft,
                           bool& imu,
                           bool& motor_side,
                           Sync s = Sync::All){

        pos = (s == Sync::Position) || (s == Sync::All);
        vel = (s == Sync::Velocity) || (s == Sync::All);
        acc = (s == Sync::Acceleration) || (s == Sync::All);
        eff = (s == Sync::Effort) || (s == Sync::All);
        k = (s == Sync::Stiffness) || (s == Sync::Impedance) || (s == Sync::All);
        d = (s == Sync::Damping) || (s == Sync::Impedance) || (s == Sync::All);
        ft = (s == Sync::ForceTorque) || (s == Sync::Sensors) || (s == Sync::All);
        imu = (s == Sync::Imu) || (s == Sync::Sensors) || (s == Sync::All);
        motor_side = (s == Sync::MotorSide);
        
        
        

    }

    template <typename... Flags>
    inline void parseFlags(bool& pos,
                           bool& vel,
                           bool& acc,
                           bool& eff,
                           bool& k,
                           bool& d,
                           bool& ft,
                           bool& imu,
                           bool& motor_side,
                           Sync s, Flags... rest){
        

        parseFlags(pos, vel, acc, eff, k, d, ft, imu, motor_side, rest...);

        pos = pos || (s == Sync::Position) || (s == Sync::All);
        vel = vel || (s == Sync::Velocity) || (s == Sync::All);
        acc = acc || (s == Sync::Acceleration) || (s == Sync::All);
        eff = eff || (s == Sync::Effort) || (s == Sync::All);
        k = k || (s == Sync::Stiffness) || (s == Sync::Impedance) || (s == Sync::All);
        d = d || (s == Sync::Damping) || (s == Sync::Impedance) || (s == Sync::All);
        ft = ft || (s == Sync::ForceTorque) || (s == Sync::Sensors) || (s == Sync::All);
        imu = imu || (s == Sync::Imu) || (s == Sync::Sensors) || (s == Sync::All);
        motor_side = motor_side || (s == Sync::MotorSide);
        

    }



    class FlagsParser {

    public:
        

        FlagsParser(){
            load_flags();
        }

        template <typename... Flags>
        void load_flags(Flags... flags)
        {
            pos = false;
            vel = false;
            acc = false;
            eff = false;
            k = false;
            d = false;
            ft = false;
            imu = false;
            mot = false;

            parseFlags(pos,
                vel,
                acc,
                eff,
                k,
                d,
                ft,
                imu,
                mot,
                flags...
            );
            
            if(mot && !pos && !vel)
            {
                throw std::runtime_error("Sync::MotorSide must be used together with Position OR Velocity!");
            }

        }

        bool position_flag() const { return pos; }
        bool velocity_flag() const { return vel; }
        bool acceleration_flag() const { return acc; }
        bool effort_flag() const { return eff; }
        bool stiffness_flag() const { return k; }
        bool damping_flag() const { return d; }
        bool ft_flag() const { return ft; }
        bool imu_flag() const { return imu; }
        bool motor_side_flag() const { return mot; }

    private:

        bool pos, vel, acc, eff, k, d, ft, imu, mot;

    };

}
#endif