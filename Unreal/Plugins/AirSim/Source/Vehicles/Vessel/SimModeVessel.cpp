// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#include "SimModeVessel.h"
#include "UObject/ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "VesselPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "vehicles/vessel/api/VesselRpcLibServer.hpp"


void ASimModeVessel::BeginPlay()
{
	Super::BeginPlay();

	//let base class setup physics world
	initializeForPlay();
}

//void ASimModeVessel::initializePauseState()
//{
//	pause_period_ = 0;
//	pause_period_start_ = 0;
//	pause(false);
//}

void ASimModeVessel::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	//stop physics thread before we dismantle
	stopAsyncUpdator();

	Super::EndPlay(EndPlayReason);
}

//bool ASimModeVessel::isPaused() const
//{
//	return current_clockspeed_ == 0;
//}
//
//void ASimModeVessel::pause(bool is_paused)
//{
//	if (is_paused)
//		current_clockspeed_ = 0;
//	else
//		current_clockspeed_ = getSettings().clock_speed;
//
//	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
//}
//
//void ASimModeVessel::continueForTime(double seconds)
//{
//	pause_period_start_ = ClockFactory::get()->nowNanos();
//	pause_period_ = seconds;
//	pause(false);
//}


void ASimModeVessel::setupClockSpeed()
{
	typedef msr::airlib::ClockFactory ClockFactory;

	float clock_speed = getSettings().clock_speed;

	//setup clock in ClockFactory
	std::string clock_type = getSettings().clock_type;

	if (clock_type == "ScalableClock") {
		//scalable clock returns interval same as wall clock but multiplied by a scale factor
		ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
	}
	else if (clock_type == "SteppableClock") {
		//steppable clock returns interval that is a constant number irrespective of wall clock
		//we can either multiply this fixed interval by scale factor to speed up/down the clock
		//but that would cause vehicles like quadrotors to become unstable
		//so alternative we use here is instead to scale control loop frequency. The downside is that
		//depending on compute power available, we will max out control loop frequency and therefore can no longer
		//get increase in clock speed

		//Approach 1: scale clock period, no longer used now due to quadrotor instability
		//ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
		//static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));

		//Approach 2: scale control loop frequency if clock is speeded up
		if (clock_speed >= 1) {
			ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
				static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9))); //no clock_speed multiplier

			setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
		}
		else {
			//for slowing down, this don't generate instability
			ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
				static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
		}
	}
	else
		throw std::invalid_argument(common_utils::Utils::stringf(
			"clock_type %s is not recognized", clock_type.c_str()));
}

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeVessel::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
	return ASimModeBase::createApiServer();
#else
	return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::VesselRpcLibServer(
		getApiProvider(), getSettings().api_server_address, getSettings().api_port));
	//return nullptr;
#endif
}

void ASimModeVessel::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
	UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
}

bool ASimModeVessel::isVehicleTypeSupported(const std::string& vehicle_type) const
{
	return (
		vehicle_type == AirSimSettings::kVehicleTypeMilliAmpere ||
		vehicle_type == AirSimSettings::kVehicleTypeQiuxin ||
		vehicle_type == AirSimSettings::kVehicleTypeCybership2 ||
		vehicle_type == AirSimSettings::kVehicleTypeMariner
		// vehicle_type == AirSimSettings::kVehicleTypeTanker 
		// vehicle_type == AirSimSettings::kVehicleTypeMilliAmpereCurrent
		);
}

std::string ASimModeVessel::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
	//decide which derived BP to use
	std::string pawn_path = vehicle_setting.pawn_path;
	if (pawn_path == "") {
		pawn_path = "DefaultVessel";
	}

	return pawn_path;
}

PawnEvents* ASimModeVessel::getVehiclePawnEvents(APawn* pawn) const
{
	return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeVessel::getVehiclePawnCameras(
	APawn* pawn) const
{
	return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
}
void ASimModeVessel::initializeVehiclePawn(APawn* pawn)
{
	auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
	vehicle_pawn->initializeForBeginPlay();
}
std::unique_ptr<PawnSimApi> ASimModeVessel::createVehicleSimApi(
	const PawnSimApi::Params& pawn_sim_api_params) const

{
	// auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn_sim_api_params.pawn);
	auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new VesselPawnSimApi(pawn_sim_api_params));
	// 	vehicle_pawn->getKeyBoardControls(), vehicle_pawn->getVehicleMovementComponent()));
	vehicle_sim_api->initialize();
	// vehicle_sim_api->reset();
	return vehicle_sim_api;
}
msr::airlib::VehicleApiBase* ASimModeVessel::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
	const PawnSimApi* sim_api) const
{
	const auto vessel_sim_api = static_cast<const VesselPawnSimApi*>(sim_api);
	return vessel_sim_api->getVehicleApi();
}
