// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef air_VesselRpcLibClient_hpp
#define air_VesselRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "vehicles/vessel/api/VesselApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "common/ImageCaptureBase.hpp"


namespace msr {
    namespace airlib {

        class VesselRpcLibClient : public RpcLibClientBase {
        public:
            VesselRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

            void setVesselControls(const VesselApiBase::VesselControls& controls, const std::string& vehicle_name = "");
			void setDisturbanceControls(const VesselApiBase::DisturbanceControls& controls, const std::string& vehicle_name = "");
            VesselApiBase::VesselState getVesselState(const std::string& vehicle_name = "");
            // VesselApiBase::VesselState getActuation(const std::string& vehicle_name = "", const int actuator);

            virtual ~VesselRpcLibClient();    //required for pimpl
        };

    }
} //namespace
#endif
