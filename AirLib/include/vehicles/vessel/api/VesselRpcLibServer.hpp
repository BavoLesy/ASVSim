// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef air_VesselRpcLibServer_hpp
#define air_VesselRpcLibServer_hpp

#ifndef AIRLIB_NO_RPC

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/vessel/api/VesselApiBase.hpp"

namespace msr { namespace airlib {


class VesselRpcLibServer : public RpcLibServerBase {
public:
    
    VesselRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = RpcLibPort);
    virtual ~VesselRpcLibServer();

protected:
    virtual VesselApiBase* getVehicleApi(const std::string& vehicle_name) override
    {
        return static_cast<VesselApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
    }

};

#endif
}} //namespace
#endif
