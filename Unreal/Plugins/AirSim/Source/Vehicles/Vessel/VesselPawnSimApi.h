// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#pragma once

#include "CoreMinimal.h"
#include "PawnSimApi.h"
#include "vehicles/vessel/Vessel.hpp"
#include "vehicles/vessel/VesselParams.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/common_utils/UniqueValueMap.hpp" 
#include "VesselPawnEvents.h"
#include <future>


class VesselPawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::Vessel Vessel;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;
    typedef VesselPawnEvents::RudderInfo RudderInfo;

public:
    virtual void initialize() override;

    virtual ~VesselPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    VesselPawnSimApi(const Params& params);
    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;
    

    //PhysicsBody interface
    //this just wrapped around MultiRotor physics body
    virtual void resetImplementation() override;
    virtual void update(float delta = 0) override;
    virtual void reportState(StateReporter& reporter) override;
    virtual UpdatableObject* getPhysicsBody() override;

    virtual void setPose(const Pose& pose, bool ignore_collision) override;
    virtual void pawnTick(float dt) override;

    msr::airlib::VesselApiBase* getVehicleApi() const
    {
        return vehicle_api_.get();
    }

    virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
    {
        return vehicle_api_.get();
    }
private:
    std::unique_ptr<msr::airlib::VesselApiBase> vehicle_api_;
    std::unique_ptr<msr::airlib::VesselParams> vehicle_params_;
    std::unique_ptr<msr::airlib::AbstractHydrodynamics> hydrodynamics_;

    std::unique_ptr<Vessel> phys_vehicle_;
    unsigned int rotor_count_;
    std::vector<RudderInfo> rudder_info;

    //show info on collision response from physics engine
    msr::airlib::CollisionResponse collision_response;

    VesselPawnEvents* pawn_events_;

    //when pose needs to set from non-physics thread, we set it as pending
    bool pending_pose_collisions_;
    enum class PendingPoseStatus {
        NonePending, RenderPending
    } pending_pose_status_;
    Pose pending_phys_pose_; //force new pose through API

    //reset must happen while World is locked so its async task initiated from API thread
    bool reset_pending_;
    bool did_reset_;
    std::packaged_task<void()> reset_task_;

    Pose last_phys_pose_; //for trace lines showing vehicle path
    std::vector<std::string> vehicle_api_messages_;
    // AirSimSettings::VehicleSetting* stub_settings_;
    const Params* params_;

    //controls
    msr::airlib::VesselApiBase::VesselControls joystick_controls_;
    msr::airlib::VesselApiBase::VesselControls current_controls_;

};
