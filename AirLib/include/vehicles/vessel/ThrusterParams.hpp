// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_ThrusterParams_hpp
#define msr_airlib_ThrusterParams_hpp


#include "common/Common.hpp"

namespace msr {
    namespace airlib {

        struct ThrusterParams {
            real_T thrust_signal_filter_tc = 2.0f;        // time constant for low pass filter
            real_T angle_signal_filter_tc = 0.5f;       // time constant for low pass filter
            real_T max_thrust = calculate_max_thrust();   // N
            real_T max_rudder_angle = M_PI / 6;             // Radians (=30 degrees)
            real_T l_x = -1.8;
            real_T l_y = 0;

        private:
            constexpr real_T calculate_max_thrust() {
                // TODO: implement a more accurate thrust model
                return 500; //N
            }

        };
    }
} //namespace
#endif
