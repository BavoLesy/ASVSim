// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef airsim_core_large_vessel_engine_hpp
#define airsim_core_large_vessel_engine_hpp

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include "common/CommonStructs.hpp"
#include "common/SteppableClock.hpp"
#include <cinttypes>

namespace msr {
    namespace airlib {

        struct VesselStateRK4 {
            Vector3r position; // N (x), E (y), Heading
            Vector3r nu;       // [u, v, r]  
        };

        struct VesselDerivativeRK4 {
            Vector3r dposition;
            Vector3r dnu;
        };

        class LargeVesselEngine : public PhysicsEngineBase {

        public:
            LargeVesselEngine()
            {
            }

            //*** Start: UpdatableState implementation ***//
            virtual void resetImplementation() override
            {
                PhysicsEngineBase::resetImplementation();

                for (PhysicsBody* body_ptr : *this) {
                    initPhysicsBody(body_ptr);
                }
            }

            virtual void insert(PhysicsBody* body_ptr) override
            {
                PhysicsEngineBase::insert(body_ptr);

                initPhysicsBody(body_ptr);
            }

            virtual void update(float delta = 0) override
            {
                PhysicsEngineBase::update(delta);

                for (PhysicsBody* body_ptr : *this) {
                    updatePhysics(*body_ptr);
                }
            }
            virtual void reportState(StateReporter& reporter) override
            {
                for (PhysicsBody* body_ptr : *this) {
                    reporter.writeValue("Phys", debug_string_.str());
                    reporter.writeValue("Force (world)", body_ptr->getWrench().force);
                    reporter.writeValue("Torque (body)", body_ptr->getWrench().torque);
                }
                //call base
                UpdatableObject::reportState(reporter);
            }
            //*** End: UpdatableState implementation ***//

        private:
            void initPhysicsBody(PhysicsBody* body_ptr)
            {
                body_ptr->last_kinematics_time = clock()->nowNanos();
            }

            void updatePhysics(PhysicsBody& body)
            {

                TTimeDelta dt = clock()->updateSince(body.last_kinematics_time);
                body.lock();

                //get current kinematics state of the body - this state existed since last dt seconds
                Kinematics::State kinematics = body.getKinematics();
                VesselStateRK4 current_state;
				current_state.position = Vector3r(kinematics.pose.position.x(), kinematics.pose.position.y(), toYaw(kinematics.pose.orientation));
				current_state.nu = Vector3r(kinematics.twist.linear.x(), kinematics.twist.linear.y(), kinematics.twist.angular.z());

                // Update twist and acceleration with eta_dot and nu_dot
				VesselDerivativeRK4 dstate;
                body.updateKinematics(kinematics);
				kinematics = body.getKinematics(); // Get updated kinematics after updateKinematics call
                dstate.dposition = Vector3r(kinematics.twist.linear.x(), kinematics.twist.linear.y(), kinematics.twist.angular.z());
                dstate.dnu = Vector3r(kinematics.accelerations.linear.x(), kinematics.accelerations.linear.y(), kinematics.accelerations.angular.z());

                // Euler step
				euler_step(current_state, dstate, static_cast<float>(dt));

				// Update internal state
				kinematics.pose.position = Vector3r(current_state.position.x(), current_state.position.y(), 0.0f);
				kinematics.pose.orientation.w() = std::cos(current_state.position.z() / 2.0f);
				kinematics.pose.orientation.z() = std::sin(current_state.position.z() / 2.0f);
				kinematics.twist.linear = Vector3r(current_state.nu.x(), current_state.nu.y(), 0.0f);
				kinematics.twist.angular = Vector3r(0.0f, 0.0f, current_state.nu.z());


				// Update the kinematics of the body
				body.PhysicsBody::updateKinematics(kinematics);

                body.unlock();
            }



            void throttledLogOutput(const std::string& msg, double seconds)
            {
                TTimeDelta dt = clock()->elapsedSince(last_message_time);
                const real_T dt_real = static_cast<real_T>(dt);
                if (dt_real > seconds)
                {
                    Utils::log(msg);
                    last_message_time = clock()->nowNanos();
                }
            }

            static void euler_step(
                VesselStateRK4& state,
                const VesselDerivativeRK4& deriv,
                const float dt
            )
            {
                state.position += dt * deriv.dposition;
                state.nu += dt * deriv.dnu;
            }

            // Returns yaw in radians
            static float toYaw(const Quaternionr& q) {
                double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
                double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
                return std::atan2(siny_cosp, cosy_cosp);
            }
        private:
            std::stringstream debug_string_;
            TTimePoint last_message_time;
        };
    }
} //namespace


#endif