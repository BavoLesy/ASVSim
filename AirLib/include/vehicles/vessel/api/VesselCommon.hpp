// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef air_VesselCommon_hpp
#define air_VesselCommon_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"

namespace msr {  namespace airlib {

struct VesselState {
    CollisionInfo collision;
    Kinematics::State kinematics_estimated;
    GeoPoint gps_location;
    uint64_t timestamp;
    RCData rc_data;

    VesselState()
    {}
    VesselState(const CollisionInfo& collision_val, const Kinematics::State& kinematics_estimated_val, 
        const GeoPoint& gps_location_val, uint64_t timestamp_val,
        const RCData& rc_data_val)
        : collision(collision_val), kinematics_estimated(kinematics_estimated_val),
        gps_location(gps_location_val), timestamp(timestamp_val),
        rc_data(rc_data_val)
    {
    }

    //shortcuts
    const Vector3r& getPosition() const
    {
        return kinematics_estimated.pose.position;
    }
    const Quaternionr& getOrientation() const
    {
        return kinematics_estimated.pose.orientation;
    }
};

}} //namespace
#endif