// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#pragma once

#include "vehicles/vessel/api/VesselApiBase.hpp"
#include "vehicles/vessel/VesselParams.hpp"

#include "physics/Kinematics.hpp"
#include "VesselPawn.h"
#include "common/Common.hpp"


class VesselApi : public msr::airlib::VesselApiBase {
public:
	typedef msr::airlib::AirSimSettings AirSimSettings;
	typedef msr::airlib::Kinematics Kinematics;
	typedef msr::airlib::Environment Environment;
	typedef msr::airlib::SensorFactory SensorFactory;

	VesselApi(AVesselPawn* const pawn, const AirSimSettings::VehicleSetting* vehicle_setting,
		std::shared_ptr<SensorFactory> sensor_factory,
		const Kinematics::State& state, const Environment& environment);

	typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

	// virtual void setVesselControls(const VesselApiBase::VesselControls& controls) override;

	virtual void resetImplementation() override;
	virtual void update(float delta = 0) override;

	virtual msr::airlib::GeoPoint getHomeGeoPoint() const override;
	virtual void enableApiControl(bool is_enabled) override;
	virtual bool isApiControlEnabled() const override;
	virtual bool armDisarm(bool arm) override;
	virtual VesselState getVesselState() const override;

	// virtual const VesselApiBase::VesselControls& getVesselControls() const override;

	virtual ~VesselApi();

private:
	bool api_control_enabled_ = false;
	VesselControls last_controls_;
	const AVesselPawn* pawn_;
	const msr::airlib::Kinematics::State* pawn_kinematics_;
	msr::airlib::GeoPoint  home_geopoint_;
	const Kinematics::State& state_;
};