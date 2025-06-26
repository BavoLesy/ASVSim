// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef airsimcore_abstractTau_hpp
#define airsimcore_abstractTau_hpp

#include <limits>
#include <functional>
#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include "common/FirstOrderFilter.hpp"
#include "ThrusterParams.hpp"
#include "common/SteppableClock.hpp"
#include "AbstractHydrodynamics.hpp"

namespace msr {
    namespace airlib {

        // Rudder gets control signal as input (Desired normalized thrust (N) and normalized desired rudder angle (radians)) 
        // which causes change in rotation speed and turning direction and ultimately produces force (surge) and torque (yaw) 
        // as output. Currently, we consider an under_actuated vessel with 2 controllable degrees.
        class AbstractTau : public PhysicsBodyVertex {

        public: // types
            struct Output {
                real_T thrust_signal_input = 0;
                real_T thrust_signal_filtered = 0;
                real_T angle_signal_input = 0;
                real_T angle_signal_filtered = 0;
                real_T surge_force = 0;
                real_T sway_force = 0;
                real_T torque = 0;
            };

        public: //methods

            virtual void initialize(const Vector3r& position, const Vector3r& normal)
            {
                PhysicsBodyVertex::initialize(position, normal);   //call base initializer
            }

            // For actuators
            virtual void setControlSignal(real_T force_signal, real_T angle_signal) { ; }

            // For internal system
            virtual void setComputationMethod(std::function<Vector3r()> func) { ; }

            // No override allowed, should happen by setting output_
            Output getOutput() const {
                return output_;
            }

            virtual ~AbstractTau() = default;

            void updateGlobalOrientation(const Quaternionr& orientation)
            {
                global_heading_ = toYaw(orientation);
            }

        protected:
            Output output_;
            float global_heading_;

        private:
            // Returns yaw in radians
            static float toYaw(const Quaternionr& q) {
                double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
                double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
                return std::atan2(siny_cosp, cosy_cosp);
            }
        };
    }
} //namespace
#endif
