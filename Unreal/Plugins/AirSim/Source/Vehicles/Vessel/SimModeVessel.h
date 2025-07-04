// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#pragma once

#include "CoreMinimal.h"
#include "VesselPawn.h"
#include "common/Common.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "SimMode/SimModeWorldBase.h"

#include "SimModeVessel.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API ASimModeVessel : public ASimModeWorldBase
{
	GENERATED_BODY()
	
public:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason);

	// virtual bool isPaused() const override;
	// virtual void pause(bool is_paused) override;
	// virtual void continueForTime(double seconds) override;

//private:
//	typedef msr::airlib::ClockFactory ClockFactory;
//	typedef common_utils::Utils Utils;
//	typedef msr::airlib::TTimePoint TTimePoint;
//	typedef msr::airlib::TTimeDelta TTimeDelta;
//	typedef AVesselPawn TVehiclePawn;
//	typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
//	typedef msr::airlib::VectorMath VectorMath;
//	typedef msr::airlib::Vector3r Vector3r;

private:
	// void initializePauseState();

protected:
	virtual void setupClockSpeed() override;
	virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;
	virtual void getExistingVehiclePawns(TArray<AActor*>& pawns) const override;
	virtual bool isVehicleTypeSupported(const std::string& vehicle_type) const override;
	virtual std::string getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const override;
	virtual PawnEvents* getVehiclePawnEvents(APawn* pawn) const override;
	virtual const common_utils::UniqueValueMap<std::string, APIPCamera*> getVehiclePawnCameras(APawn* pawn) const override;
	virtual void initializeVehiclePawn(APawn* pawn) override;
	virtual std::unique_ptr<PawnSimApi> createVehicleSimApi(
		const PawnSimApi::Params& pawn_sim_api_params) const override;
	virtual msr::airlib::VehicleApiBase* getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
		const PawnSimApi* sim_api) const override;

//private:
//	std::atomic<float> current_clockspeed_;
//	std::atomic<TTimeDelta> pause_period_;
//	std::atomic<TTimePoint> pause_period_start_;

private:
	typedef AVesselPawn TVehiclePawn;
};
