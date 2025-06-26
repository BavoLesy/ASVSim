// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef airsimcore_disturbanceWind_hpp
#define airsimcore_disturbanceWind_hpp

#include <limits>
#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include "common/FirstOrderFilter.hpp"
#include "AbstractTau.hpp"
#include "VesselParams.hpp"
#include "common/VectorMath.hpp"

namespace msr {
    namespace airlib {

        // Rudder gets control signal as input (Desired normalized thrust (N) and normalized desired rudder angle (radians)) 
        // which causes change in rotation speed and turning direction and ultimately produces force (surge) and torque (yaw) 
        // as output. Currently, we consider an under_actuated vessel with 2 controllable degrees.
        class DisturbanceWind : public AbstractTau {

            using AbstractTau::initialize;  // No overloading across scopes in c++

        public: //methods

            DisturbanceWind()
            {
                //allow default constructor with later call for initialize
            }

            DisturbanceWind(const Vector3r& position, const Vector3r& normal, const VesselParams* params, const Environment* environment, uint id = -1)
            {
                initialize(position, normal, params, environment, id);
            }

            void initialize(const Vector3r& position, const Vector3r& normal,
                const VesselParams* params, const Environment* environment, uint id = -1)
            {
                id_ = id;
                params_ = params;
                environment_ = environment;

                AbstractTau::initialize(position, normal);   //call base initializer
            }

            //0 to 1 - will be scaled to 0 to max_speed
            void setControlSignal(real_T force_signal, real_T angle_signal) override
            {
				wind_force_ = force_signal;
				wind_angle_ = angle_signal;
            }


            //*** Start: UpdatableState implementation ***//
            virtual void resetImplementation() override
            {
                PhysicsBodyVertex::resetImplementation();

                //update environmental factors before we call base
                updateEnvironmentalFactors();

                setOutput(output_, 0.0, 0.0, 0.0, params_);
                time_ = 0.0;
            }
            
            virtual void update(volatile float delta) override
            {
                //update environmental factors before we call base
                updateEnvironmentalFactors();

                //this will in turn call setWrench
                PhysicsBodyVertex::update(delta);

                //update our state
				time_ = clock()->nowNanos();
                relative_angle_ = wind_angle_ - global_heading_;
                setOutput(output_, time_, wind_force_, relative_angle_, params_);
            }

            virtual void reportState(StateReporter& reporter) override
            {
                reporter.writeValue("Surge-force", output_.surge_force);
                reporter.writeValue("Sway-force", output_.sway_force);
                reporter.writeValue("torque", output_.torque);
				reporter.writeValue("Relative wind angle", relative_angle_);
            }
            //*** End: UpdatableState implementation ***//


        protected:
            virtual void setWrench(Wrench& wrench) override
            {
                // This represents the rudder at an angle
                wrench.force = Vector3r(output_.surge_force, output_.sway_force, 0.0f);
                wrench.torque = Vector3r(0.0f, 0.0f, output_.torque);
            }

        private: //methods
            static void setOutput(Output& output, double delta, float force, float relative_angle, const VesselParams* params)
            {
				output.surge_force = force * std::cos(relative_angle) * (1.0+std::sin(delta));
                output.sway_force = -force * std::sin(relative_angle) * (1.0+std::sin(delta));
                output.torque = force * std::sin(2.0*(relative_angle)) * (1.0+std::sin(delta)) * params->getParams().length * 0.25;
            }

            void updateEnvironmentalFactors()
            {
                // Pass
            }

        private: //fields
            uint id_; //only used for debug messages
            const VesselParams* params_ = nullptr;
            const Environment* environment_ = nullptr;
			float wind_angle_ = 0.0f;   // [rad]
			float wind_force_ = 0.0f;   // [N]
			float relative_angle_ = 0.0f; // [rad]
			double time_ = 0.0;
        };


    }
} //namespace
#endif
