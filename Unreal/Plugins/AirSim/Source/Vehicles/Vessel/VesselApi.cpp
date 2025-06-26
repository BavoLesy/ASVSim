// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#include "VesselApi.h"
#include "AirBlueprintLib.h"

VesselApi::VesselApi(AVesselPawn* const pawn, const AirSimSettings::VehicleSetting* vehicle_setting,
	std::shared_ptr<SensorFactory> sensor_factory,
	const Kinematics::State& state, const Environment& environment) :
	msr::airlib::VesselApiBase(vehicle_setting, sensor_factory, state, environment),
	pawn_(pawn), state_(state)
{
	;
}

bool VesselApi::armDisarm(bool arm)
{
	//todo?: implement arming for vessel
	unused(arm);
	return true;
}

VesselApi::VesselState VesselApi::getVesselState() const
{
	VesselState state;
	state.kinematics_estimated = state_;
	state.timestamp = clock()->nowNanos();
	return state;
}

void VesselApi::resetImplementation()
{
	msr::airlib::VesselApiBase::resetImplementation();

	last_controls_ = VesselControls();
}

void VesselApi::update(float delta)
{
	msr::airlib::VesselApiBase::update(delta);
}

msr::airlib::GeoPoint VesselApi::getHomeGeoPoint() const
{
	return home_geopoint_;
}

void VesselApi::enableApiControl(bool is_enabled)
{
	if (api_control_enabled_ != is_enabled) {
		last_controls_ = VesselControls();
		api_control_enabled_ = is_enabled;
	}
}

bool VesselApi::isApiControlEnabled() const
{
	return api_control_enabled_;
}

VesselApi::~VesselApi() = default;
