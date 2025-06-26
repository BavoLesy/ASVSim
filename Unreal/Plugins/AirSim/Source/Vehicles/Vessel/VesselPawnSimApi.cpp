// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#include "VesselPawnSimApi.h"
#include "VesselApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/vessel/VesselParamsFactory.hpp"
#include "Vehicles/vessel/HydrodynamicsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "VesselPawn.h"
#include <exception>

using namespace msr::airlib;

VesselPawnSimApi::VesselPawnSimApi(const Params& params)
    : PawnSimApi(params)
{
    pawn_events_ = static_cast<VesselPawnEvents*>(params.pawn_events);
    params_ = &params;
}

void VesselPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    // vehicle_params_ = std::make_unique<MilliAmpereParams>();// MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_params_ = VesselParamsFactory::createConfig(getVehicleSetting());
    vehicle_api_ = std::make_unique<VesselApi>(static_cast<AVesselPawn*>(params_->pawn), getVehicleSetting(), sensor_factory, *getGroundTruthKinematics(), *getGroundTruthEnvironment());
    hydrodynamics_ = HydrodynamicsFactory::createHydrodynamics(getVehicleSetting());
    hydrodynamics_->setVesselParameters(vehicle_params_.get());
    //setup physics vehicle
    phys_vehicle_ = std::make_unique<Vessel>(vehicle_params_.get(), hydrodynamics_.get(), vehicle_api_.get(),
                                getKinematics(), getEnvironment());
    rotor_count_ = phys_vehicle_->wrenchVertexCount();
    rudder_info.assign(rotor_count_, RudderInfo());

    //initialize private vars
    last_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);
}

void VesselPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

void VesselPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();
    phys_vehicle_->setCollisionInfo(collision_info);

    last_phys_pose_ = phys_vehicle_->getPose();

    collision_response = phys_vehicle_->getCollisionResponseInfo();

    //update rotor poses
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        const auto& rudder_output = phys_vehicle_->getRudderOutput(i);
        RudderInfo* info = &rudder_info[i];
        info->thrust = rudder_output.surge_force;
        info->torque = rudder_output.torque;
        info->angle_control_filtered = rudder_output.angle_signal_filtered;
        info->thrust_control_filtered = rudder_output.thrust_signal_filtered;
    }

    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
}

void VesselPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {
        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ == PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"),
        FString::FromInt(collision_response.collision_count_non_resting), LogDebugLevel::Informational);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception& e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    // DEBUG
    auto& signal = pawn_events_->getActuatorSignal();
    // signal.emit(rudder_info);
    // pawn_events_->getActuatorSignal().emit(rudder_info);
    int debug = 4;
}

void VesselPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    phys_vehicle_->lock();
    phys_vehicle_->setPose(pose);
    phys_vehicle_->setGrounded(false);
    phys_vehicle_->unlock();
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderPending;
}

//*** Start: UpdatableState implementation ***//
void VesselPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    vehicle_api_->reset();
    phys_vehicle_->reset();
    vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void VesselPawnSimApi::update(float delta)
{
    //environment update for current position
    PawnSimApi::update(delta);

    //update forces on vertices
    phys_vehicle_->update(delta);

    //update to controller must be done after kinematics have been updated by physics engine
}

void VesselPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    phys_vehicle_->reportState(reporter);
}

VesselPawnSimApi::UpdatableObject* VesselPawnSimApi::getPhysicsBody()
{
    return phys_vehicle_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

