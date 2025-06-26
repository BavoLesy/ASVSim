// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "PawnEvents.h"
#include "common/Common.hpp"
#include "common/common_utils/Signal.hpp"


class VesselPawnEvents : public PawnEvents {
public: //types
    typedef msr::airlib::real_T real_T;
    struct RudderInfo {
        real_T thrust = 0;
        real_T thrust_control_filtered = 0;
        real_T torque = 0;
        real_T angle_control_filtered = 0;
    };

    typedef common_utils::Signal<const std::vector<RudderInfo>&> ActuatorsSignal;

public:
    ActuatorsSignal& getActuatorSignal();

private:
    ActuatorsSignal actuator_signal_;
};
