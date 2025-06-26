// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_vessel_hpp
#define msr_airlib_vessel_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"
#include "VesselParams.hpp"
#include "Thruster.hpp"
#include "AbstractTau.hpp"
#include "DisturbanceWind.hpp"
#include "AbstractHydrodynamics.hpp"
#include "HydrodynamicsWrench.hpp"

namespace msr {
    namespace airlib {

        class Vessel : public PhysicsBody {
        public:
            Vessel(VesselParams* params, AbstractHydrodynamics* hydrodynamics, VesselApiBase* vessel_api,
                Kinematics* kinematics, Environment* environment)
                : params_(params), hydrodynamics_(hydrodynamics), vessel_api_(vessel_api)
            {
                initialize(kinematics, environment);
            }

            //*** Start: UpdatableState implementation ***//
            virtual void resetImplementation() override
            {
                //reset rotors, kinematics and environment
                PhysicsBody::resetImplementation();

                //reset sensors last after their ground truth has been reset
                resetSensors();

                //Kinematics::State initialized_state = getKinematics();
                //real_T speed = params_->getParams().design_speed.value_or(0.0f);
                //initialized_state.twist.linear = Vector3r(speed, 0.0f, 0.0f);
                //PhysicsBody::updateKinematics(initialized_state);
            }

            virtual void update(float delta = 0) override
            {
                //update forces on vertices that we will use next
                PhysicsBody::update(delta);

                //Note that controller gets updated after kinematics gets updated in updateKinematics
                //otherwise sensors will have values from previous cycle causing lags which will appear
                //as crazy jerks whenever commands like velocity is issued
            }
            virtual void reportState(StateReporter& reporter) override
            {
                //call base
                PhysicsBody::reportState(reporter);

                reportSensors(*params_, reporter);

                //report thruster
                short i = 0;
                for (const auto& wrench : wrenches_) {
                    reporter.startHeading("", 1);
                    reporter.writeValue("Wrench", ++i);
                    reporter.endHeading(false, 1);
                    wrench->reportState(reporter);
                }
            }
            //*** End: UpdatableState implementation ***//


            //Physics engine calls this method to set next kinematics
            virtual void updateKinematics(const Kinematics::State& kinematics) override
            {
                if (params_->getParams().large_vessel) {
                    hydrodynamics_->updateState(kinematics, Vector2r::Zero());
                    hydrodynamics_->setRudderAngle(vessel_api_->getActuation(1) - 0.5);
                    hydrodynamics_->computeDamping();
                    hydrodynamics_->computeCoriolis();
                    Kinematics::State kin_copy = getKinematics();

                    // Set eta_dot
                    kin_copy.twist.linear = Vector3r(hydrodynamics_->getEtaDot().x(), hydrodynamics_->getEtaDot().y(), 0.0f);
                    kin_copy.twist.angular = Vector3r(0.0f, 0.0f, hydrodynamics_->getEtaDot().z());

                    // Set nu_dot
                    kin_copy.accelerations.linear = Vector3r(hydrodynamics_->getNuDot().x(), hydrodynamics_->getNuDot().y(), 0.0f);
                    kin_copy.accelerations.angular = Vector3r(0.0f, 0.0f, hydrodynamics_->getNuDot().z());

                    PhysicsBody::updateKinematics(kin_copy);
                }
                else
                {
                    PhysicsBody::updateKinematics(kinematics);
                    updateSensors(*params_, getKinematics(), getEnvironment());
                    vessel_api_->update();
                    auto& global_orientation = getKinematics().pose.orientation;

                    for (auto& hydro : hydros_) {
                        hydro->updateGlobalOrientation(global_orientation);
                    }

                    int counter = 0;
                    for (auto& thruster : thrusters_) {
                        thruster->updateGlobalOrientation(global_orientation);
                        thruster->setControlSignal(vessel_api_->getActuation(counter), vessel_api_->getActuation(counter + 1));
                        counter += 2;
                    }

                    for (auto& disturbance : disturbances_) {
                        disturbance->updateGlobalOrientation(global_orientation);
                        disturbance->setControlSignal(vessel_api_->getDisturbanceAngle(DisturbanceType::Wind).first,
                            vessel_api_->getDisturbanceAngle(DisturbanceType::Wind).second);
                    }

                    Vector2r current = vessel_api_->getDisturbanceXY(DisturbanceType::Current);
                    drag_vertices_.at(0).setPosition(Vector3r(current.x(), current.y(), 0.0));

                    //transfer kinematics to hydrodynamics engine
                    TTimeDelta dt = clock()->elapsedSince(this->last_kinematics_time);
                    hydrodynamics_->updateState(kinematics, current);
                    hydrodynamics_->computeCoriolis();
                    hydrodynamics_->computeDamping();
                }
            }

            //sensor getter
            const SensorCollection& getSensors() const
            {
                return params_->getSensors();
            }

            virtual uint wrenchVertexCount() const  override
            {
                return static_cast<uint>(wrenches_.size());
            }
            virtual PhysicsBodyVertex& getWrenchVertex(uint index)  override
            {
                return *(wrenches_.at(index));
            }
            virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
            {
                return *(wrenches_.at(index));
            }

            virtual uint dragVertexCount() const override
            {
                return static_cast<uint>(drag_vertices_.size());
            }
            virtual PhysicsBodyVertex& getDragVertex(uint index)  override
            {
                return drag_vertices_.at(index);
            }
            virtual const PhysicsBodyVertex& getDragVertex(uint index) const override
            {
                return drag_vertices_.at(index);
            }

            virtual real_T getRestitution() const override
            {
                return 0.0f;
            }
            virtual real_T getFriction()  const override
            {
                return 0.0f;
            }

            AbstractTau::Output getRudderOutput(uint rotor_index) const
            {
                return wrenches_.at(2)->getOutput();
            }

            virtual ~Vessel() = default;

        private: //methods

            void initialize(Kinematics* kinematics, Environment* environment)
            {
                PhysicsBody::initialize(params_->getParams().mass, params_->getParams().mass_matrix, kinematics, environment);
                createWrenches(params_, wrenches_, environment);
                thruster_count_ = params_->getParams().rudder_params.size();

                // Damping, coriolis
                wrenches_.at(0)->setComputationMethod([this]() {return -hydrodynamics_->getDampingForce(); });
                wrenches_.at(1)->setComputationMethod([this]() {return -hydrodynamics_->getCoriolisForce(); });
                createDragVertices();
                initSensors(params_, getKinematics(), getEnvironment());

                // Create our views
                hydros_ = std::span<std::unique_ptr<AbstractTau>>(wrenches_.begin(), wrenches_.begin() + hydro_count_);
                thrusters_ = std::span<std::unique_ptr<AbstractTau>>(wrenches_.begin() + hydro_count_, wrenches_.end() - disturbance_count_);
                disturbances_ = std::span<std::unique_ptr<AbstractTau>>(wrenches_.end() - disturbance_count_, wrenches_.end());
            }

            static void createWrenches(const VesselParams* params, vector<std::unique_ptr<AbstractTau>>& wrenches, const Environment* environment)
            {
                // System
                wrenches.push_back(std::make_unique<HydroDynamicsWrench>(Vector3r(0, 0, 0), Vector3r(-1, 0, 0)));
                wrenches.push_back(std::make_unique<HydroDynamicsWrench>(Vector3r(0, 0, 0), Vector3r(-1, 0, 0)));

                // Actuators
                for (const auto& rudder_param : params->getParams().rudder_params) {
                    wrenches.push_back(std::make_unique<Thruster>(Thruster(Vector3r(0, 0, 0), Vector3r(-1, 0, 0), rudder_param, environment)));
                }

                // Disturbances (excluding current)
                wrenches.push_back(std::make_unique<DisturbanceWind>(DisturbanceWind(Vector3r(0, 0, 0), Vector3r(-1, 0, 0), params, environment)));
            }

            void reportSensors(VesselParams& params, StateReporter& reporter)
            {
                params.getSensors().reportState(reporter);
            }

            void updateSensors(VesselParams& params, const Kinematics::State& state, const Environment& environment)
            {
                unused(state);
                unused(environment);
                params.getSensors().update();
            }

            void initSensors(VesselParams* params, const Kinematics::State& state, const Environment& environment)
            {
                params->getSensors().initialize(&state, &environment);
            }

            void createDragVertices()
            {
                const auto& params = params_->getParams();
                // For compatibility reasons, we add six drag vertices representing 6 sides
                // The drag vertices are not used, as damping is already one of the wrenches.
                drag_vertices_.clear();
                for (int i = 0; i < 6; ++i) {
                    drag_vertices_.emplace_back(Vector3r::Zero(), Vector3r::Zero(), 0);
                }
            }

            void resetSensors()
            {
                params_->getSensors().reset();
            }

            inline const TTimeDelta getTimeDelta() const {
                return clock()->elapsedSince(this->last_kinematics_time);
            }

        private: //fields
            VesselParams* params_;
            AbstractHydrodynamics* hydrodynamics_;
            int thruster_count_ = 0;
            const int hydro_count_ = 2;          // coriolis, damping
            const int disturbance_count_ = 1;    // wind

            std::unique_ptr<Environment> environment_;
            VesselApiBase* vessel_api_;
            vector<PhysicsBodyVertex> drag_vertices_;
            vector<std::unique_ptr<AbstractTau>> wrenches_;

            // Views
            std::span<std::unique_ptr<AbstractTau>> hydros_;
            std::span<std::unique_ptr<AbstractTau>> thrusters_;
            std::span<std::unique_ptr<AbstractTau>> disturbances_;
        };

    }
} //namespace
#endif
