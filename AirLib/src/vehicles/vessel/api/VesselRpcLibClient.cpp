// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.


//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/vessel/api/VesselRpcLibClient.hpp"
#include "vehicles/vessel/api/VesselRpcLibServer.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <thread>
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#ifdef nil
#undef nil
#endif // nil

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/vessel/api/VesselRpcLibAdaptors.hpp"

STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning(disable : 4239))
#endif	



namespace msr {
    namespace airlib {

        VesselRpcLibServer::VesselRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port): RpcLibServerBase(api_provider, server_address, port){
            ;
        }
    
        typedef msr::airlib_rpclib::VesselRpcLibAdaptors VesselRpcLibAdaptors;

        VesselRpcLibClient::VesselRpcLibClient(const string& ip_address, uint16_t port, float timeout_sec)
            : RpcLibClientBase(ip_address, port, timeout_sec)
        {
        }

        VesselRpcLibClient::~VesselRpcLibClient()
        {}

        void VesselRpcLibClient::setVesselControls(const VesselApiBase::VesselControls& controls, const std::string& vehicle_name)
        {
            static_cast<rpc::client*>(getClient())->
                call("setVesselControls", VesselRpcLibAdaptors::VesselControls(controls), vehicle_name);
        }

		void VesselRpcLibClient::setDisturbanceControls(const VesselApiBase::DisturbanceControls& controls, const std::string& vehicle_name)
		{
			static_cast<rpc::client*>(getClient())->
				call("setDisturbanceControls", VesselRpcLibAdaptors::DisturbanceControls(controls), vehicle_name);
		}
        
        VesselApiBase::VesselState VesselRpcLibClient::getVesselState(const std::string& vehicle_name)
        {
            return static_cast<rpc::client*>(getClient())->
                call("getVesselState", vehicle_name).as<VesselRpcLibAdaptors::VesselState>().to();
        }

        /*VesselApiBase::VesselState VesselRpcLibClient::getActuation(const std::string& vehicle_name, const int actuator)
        {
            return static_cast<rpc::client*>(getClient())->
                call("getActuation", vehicle_name, actuator).as<VesselRpcLibAdaptors::VesselState>().to();
        }*/


    }
} //namespace

#endif
#endif
