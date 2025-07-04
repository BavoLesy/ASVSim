// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef airsim_core_VesselEngine_hpp
#define airsim_core_VesselEngine_hpp

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

        class VesselEngine : public PhysicsEngineBase {

        public:
            VesselEngine()
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
                const Kinematics::State& current = body.getKinematics();
                Kinematics::State next;
                Wrench next_wrench;

                //first compute the response as if there was no collision
                //this is necessary to take in to account forces and torques generated by body
                getNextKinematicsNoCollision(dt, body, current, next, next_wrench);

                // TODO: add collision detection if collision

                body.setWrench(next_wrench);
                
                body.updateKinematics(next);

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

            static Wrench getBodyWrench(const PhysicsBody& body, const Quaternionr& orientation)
            {
                //set wrench sum to zero
                Wrench wrench = Wrench::zero();

                //calculate total force on rigid body's center of gravity
                for (uint i = 0; i < body.wrenchVertexCount(); ++i) {
                    //aggregate total
                    const PhysicsBodyVertex& vertex = body.getWrenchVertex(i);
                    const auto& vertex_wrench = vertex.getWrench();
                    wrench.force += vertex_wrench.force;

                    // Torque due to force applies farther than COG
                    // tau = r X F                    
                    // wrench.torque += vertex.getPosition().cross(vertex_wrench.force);

                    // Correct torque is already calculated within the rudder
                    wrench.torque += vertex_wrench.torque;
                }

                return wrench;
            }


            static void getNextKinematicsNoCollision(TTimeDelta dt, PhysicsBody& body, const Kinematics::State& current,
                Kinematics::State& next, Wrench& next_wrench)
            {
                const real_T dt_real = static_cast<real_T>(dt);

                Vector3r avg_linear = Vector3r::Zero();
                Vector3r avg_angular = Vector3r::Zero();

                /************************* Get force and torque acting on body ************************/
                //set wrench sum to zero
                const Wrench body_wrench = getBodyWrench(body, current.pose.orientation);

                avg_linear = current.twist.linear + current.accelerations.linear * (0.5f * dt_real);
                avg_angular = current.twist.angular + current.accelerations.angular * (0.5f * dt_real);

                next_wrench = body_wrench;

                Utils::log(Utils::stringf("B-WRN %s: ", VectorMath::toString(body_wrench.force).c_str()));

                /************************* State variables *****************************************************/
                Matrix3x3r coriolis_matrix = Matrix3x3r::Zero();
                Matrix3x3r damping_matrix = Matrix3x3r::Zero();
                Vector3r nu_dot = Vector3r::Zero();

                /************************* Current state space in nu/eta notation ******************************/
                Vector3r eta = Vector3r(current.pose.position.x(), current.pose.position.y(), toYaw(current.pose.orientation));
				Vector3r nu = Vector3r(avg_linear.x(), avg_linear.y(), avg_angular.z());

                /************************* Update accelerations due to force and torque ***********************/
                Vector3r tau = Vector3r(next_wrench.force.x(), next_wrench.force.y(), next_wrench.torque.z());
                computenuDot(nu_dot, body.getInertiaInv(), tau, current.pose.orientation.z(), current.twist.angular.z(), body.getDragVertex(0).getPosition());

                //get new acceleration due to force - we'll use this acceleration in next time step
				next.accelerations.linear = Vector3r(nu_dot.x(), nu_dot.y(), 0.0f);
                next.accelerations.angular = Vector3r(0.0f, 0.0f, nu_dot.z());

                /************************* Update pose and twist after dt ************************/
                //Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
                next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);
                next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);

                //if controller has bug, velocities can increase idenfinitely 
                //so we need to clip this or everything will turn in to infinity
                if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
                    next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
                    next.accelerations.linear = Vector3r::Zero();
                }
                //
                //for disc of 1m radius which angular velocity translates to speed of light on tangent?
                if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
                    next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
                    next.accelerations.angular = Vector3r::Zero();
                }

                next.wrench.force = next_wrench.force;
                next.wrench.torque = next_wrench.torque;

                computeNextPose(dt, current.pose, avg_linear, avg_angular, next);

                Utils::log(Utils::stringf("N-VEL %s %f: ", VectorMath::toString(next.twist.linear).c_str(), dt));
                Utils::log(Utils::stringf("N-POS %s %f: ", VectorMath::toString(next.pose.position).c_str(), dt));

            }

            static void computeNextPose(TTimeDelta dt, const Pose& current_pose, const Vector3r& avg_linear, const Vector3r& avg_angular, Kinematics::State& next)
            {
                real_T dt_real = static_cast<real_T>(dt);

                // Transfrom to body frame
				Vector3r avg_linear_global = VectorMath::transformToWorldFrame(avg_linear, current_pose.orientation);

                next.pose.position = current_pose.position + avg_linear_global * dt_real;

                //use angular velocty in body frame to calculate angular displacement in last dt seconds
                real_T angle_per_unit = avg_angular.norm();
                real_T tolerance = 1e-6;
                if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f, tolerance)) {
                    //convert change in angle to unit quaternion
                    AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, avg_angular / angle_per_unit);
                    Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
                    /*
                    Proof of correctness: see FastPhysicsEngine.hpp
                    */
                    next.pose.orientation = current_pose.orientation * angle_dt_q;
                    if (VectorMath::hasNan(next.pose.orientation)) {
                        //Utils::DebugBreak();
                        Utils::log("orientation had NaN!", Utils::kLogLevelError);
                    }

                    //re-normalize quaternion to avoid accunulating error
                    next.pose.orientation.normalize();
                }
                else //no change in angle, because angular velocity is zero (normalized vector is undefined)
                    next.pose.orientation = current_pose.orientation;
            }

            static void computenuDot(Vector3r& nu_dot, const Matrix3x3r& inverse_mass_matrix, 
                                      const Vector3r& force_input, real_T psi, real_T r, const Vector3r& v_c) {
                // Dynamics
                Vector3r nu_dot_r = inverse_mass_matrix * (force_input);
                // Relative to absolute acceleration: S^T x R(\psi)^T
                // See Eqn. 9bis: https://www.sciencedirect.com/science/article/pii/S1474667017317329
                Matrix3x3r rotation;
                real_T sin = std::sin(psi);
                real_T cos = std::cos(psi);
                rotation << -sin, cos, 0,
                            -cos, sin, 0,
                            0, 0, 0;
                nu_dot = nu_dot_r + r * rotation * v_c;
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
