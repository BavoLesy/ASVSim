// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef air_VesselRpcLibAdapators_hpp
#define air_VesselRpcLibAdapators_hpp

#include <array>
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/vessel/api/VesselApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr {
    namespace airlib_rpclib {

        class VesselRpcLibAdaptors : public RpcLibAdaptorsBase {
        public:
            struct VesselControls {
				constexpr static unsigned int MAX_THRUSTER_COUNT = msr::airlib::VesselApiBase::MAX_THRUSTER_COUNT;
                std::array<float, MAX_THRUSTER_COUNT> thruster_forces = { 0.0f };
                std::array<float, MAX_THRUSTER_COUNT> thruster_angles = { 0.5f };

                MSGPACK_DEFINE_MAP(thruster_forces, thruster_angles);

                VesselControls()
                {}

                VesselControls(const msr::airlib::VesselApiBase::VesselControls& s)
                {
                    thruster_forces = s.thruster_forces;
                    thruster_angles = s.thruster_angles;
                }
                msr::airlib::VesselApiBase::VesselControls to() const
                {
                    return msr::airlib::VesselApiBase::VesselControls(thruster_forces, thruster_angles);
                }
            };

            struct DisturbanceControls {
				float wind_force;
				float wind_angle;
				//float wave_force;
				//float wave_angle;
				float current_velocity;
				float current_angle;

				MSGPACK_DEFINE_MAP(wind_force, wind_angle, current_velocity, current_angle);

				DisturbanceControls()
				{
				}

				DisturbanceControls(const msr::airlib::VesselApiBase::DisturbanceControls& s)
				{
					wind_force = s.wind_force;
					wind_angle = s.wind_angle;
					//wave_force = s.wave_force;
					//wave_angle = s.wave_angle;
                    current_velocity = s.current_velocity;
					current_angle = s.current_angle;
				}
				msr::airlib::VesselApiBase::DisturbanceControls to() const
				{
					return msr::airlib::VesselApiBase::DisturbanceControls(
						wind_force, wind_angle, current_velocity, current_angle);
				}
            };

            struct VesselState {
                float speed;
                KinematicsState kinematics_estimated;
                uint64_t timestamp;

                MSGPACK_DEFINE_MAP(speed, kinematics_estimated, timestamp);

                VesselState()
                {}

                VesselState(const msr::airlib::VesselApiBase::VesselState& s)
                {
                    timestamp = s.timestamp;
                    kinematics_estimated = s.kinematics_estimated;
                }
                msr::airlib::VesselApiBase::VesselState to() const
                {
                    return msr::airlib::VesselApiBase::VesselState(
                        kinematics_estimated.to(), timestamp);
                }
            };
        };

    }
} //namespace


#endif
