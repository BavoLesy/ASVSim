// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#pragma once

#include "GameFramework/RotatingMovementComponent.h"

#include <memory>
#include "PIPCamera.h"
#include "common/common_utils/Signal.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "VesselPawnEvents.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/SkeletalMeshComponent.h"
#include "VesselPawn.generated.h"

UCLASS(config = Game)
class AIRSIM_API AVesselPawn : public APawn
{
    GENERATED_BODY()

public:
    AVesselPawn();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
        FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface
    void initializeForBeginPlay();
    const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
    VesselPawnEvents* getPawnEvents()
    {
        return &pawn_events_;
    }
    void setRudderSpeed(const std::vector<VesselPawnEvents::RudderInfo>& rudder_infos);

private: //variables
    typedef msr::airlib::AirSimSettings AirSimSettings;

    UClass* pip_camera_class_;

    //Unreal components
    static constexpr size_t rudder_count = 1;
    UPROPERTY() APIPCamera* camera_front_left_;
    UPROPERTY() APIPCamera* camera_front_right_;
    UPROPERTY() APIPCamera* camera_front_center_;
    UPROPERTY() APIPCamera* camera_back_center_;
    UPROPERTY() APIPCamera* camera_bottom_center_;

    UPROPERTY() URotatingMovementComponent* rotating_movements_[rudder_count];

    VesselPawnEvents pawn_events_;
};
