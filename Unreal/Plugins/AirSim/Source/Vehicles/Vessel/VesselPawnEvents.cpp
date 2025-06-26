// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#include "VesselPawnEvents.h"

VesselPawnEvents::ActuatorsSignal& VesselPawnEvents::getActuatorSignal()
{
    return actuator_signal_;
}