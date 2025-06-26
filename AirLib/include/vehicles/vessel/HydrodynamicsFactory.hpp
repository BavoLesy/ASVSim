// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_vehicles_HydrodynamicsFactory_hpp
#define msr_airlib_vehicles_HydrodynamicsFactory_hpp

#include "common/AirSimSettings.hpp"
#include "vehicles/vessel/hydrodynamics/FossenCurrent.hpp"
#include "vehicles/vessel/hydrodynamics/MarinerHydrodynamics.hpp"
#include "AbstractHydrodynamics.hpp"

namespace msr
{
    namespace airlib
    {

        class HydrodynamicsFactory
        {
        public:
            static std::unique_ptr<AbstractHydrodynamics> createHydrodynamics(const AirSimSettings::VehicleSetting* vehicle_setting)
            {
                std::unique_ptr<AbstractHydrodynamics> hydrodynamics;
                std::string hydro_name = vehicle_setting->hydrodynamics->hydrodynamics_engine;

                if (hydro_name == "" || hydro_name == "FossenCurrent") {
                    hydrodynamics.reset(new FossenCurrent());
                }
                else if (hydro_name == "MarinerHydrodynamics") {
					hydrodynamics.reset(new MarinerHydrodynamics());
				}
                else
                    throw std::runtime_error(Utils::stringf(
                        "Cannot create hydrodynamics because hydrodynamics engine name '%s' is not recognized",
                        hydro_name.c_str()));

                return hydrodynamics;
            }
        };
}} //namespace

#endif