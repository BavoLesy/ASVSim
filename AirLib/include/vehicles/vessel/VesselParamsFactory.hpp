// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_vehicles_VesselParamsFactory_hpp
#define msr_airlib_vehicles_VesselParamsFactory_hpp

#include "common/AirSimSettings.hpp"
#include "vehicles/vessel/parameters/MilliAmpereParams.hpp"
#include "vehicles/vessel/parameters/QiuxinParams.hpp"
#include "vehicles/vessel/parameters/Cybership2Params.hpp"
#include "vehicles/vessel/parameters/MarinerParams.hpp"

namespace msr
{
    namespace airlib
    {

        class VesselParamsFactory
        {
        public:
            static std::unique_ptr<VesselParams> createConfig(const AirSimSettings::VehicleSetting* vehicle_setting)
            {
                std::unique_ptr<VesselParams> config;

                // Default is MilliAmpere
                if (vehicle_setting->vehicle_type == "" || vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeMilliAmpere) {
                    config.reset(new MilliAmpereParams());
                }
                else if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeQiuxin) {
                    config.reset(new QiuxinParams());
                }
                else if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeCybership2) {
                    config.reset(new Cybership2Params());
                }
				else if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeMariner) {
					config.reset(new MarinerParams());
				}
                else {
                    throw std::runtime_error(Utils::stringf(
                        "Cannot create vehicle config because vehicle name '%s' is not recognized",
                        vehicle_setting->vehicle_name.c_str()));
                }

                // config->initialize(vehicle_setting);

                return config;
            }
        };
    }
} //namespace
#endif