// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "api/RpcLibClientBase.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <functional>
#include <vector>
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

#include "api/RpcLibAdaptorsBase.hpp"

STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning(disable : 4239))
#endif

    namespace msr
{
    namespace airlib
    {

        struct RpcLibClientBase::impl
        {
            impl(const string& ip_address, uint16_t port, float timeout_sec)
                : client(ip_address, port)
            {
                // some long flight path commands can take a while, so we give it up to 1 hour max.
                client.set_timeout(static_cast<int64_t>(timeout_sec * 1.0E3));
            }

            rpc::client client;
        };

        typedef msr::airlib_rpclib::RpcLibAdaptorsBase RpcLibAdaptorsBase;

        RpcLibClientBase::RpcLibClientBase(const string& ip_address, uint16_t port, float timeout_sec)
        {
            pimpl_.reset(new impl(ip_address, port, timeout_sec));
        }

        RpcLibClientBase::~RpcLibClientBase()
        {
        }

        bool RpcLibClientBase::ping()
        {
            return pimpl_->client.call("ping").as<bool>();
        }
        RpcLibClientBase::ConnectionState RpcLibClientBase::getConnectionState()
        {
            switch (pimpl_->client.get_connection_state()) {
            case rpc::client::connection_state::connected:
                return ConnectionState::Connected;
            case rpc::client::connection_state::disconnected:
                return ConnectionState::Disconnected;
            case rpc::client::connection_state::initial:
                return ConnectionState::Initial;
            case rpc::client::connection_state::reset:
                return ConnectionState::Reset;
            default:
                return ConnectionState::Unknown;
            }
        }
        void RpcLibClientBase::enableApiControl(bool is_enabled, const std::string& vehicle_name)
        {
            pimpl_->client.call("enableApiControl", is_enabled, vehicle_name);
        }
        bool RpcLibClientBase::isApiControlEnabled(const std::string& vehicle_name) const
        {
            return pimpl_->client.call("isApiControlEnabled", vehicle_name).as<bool>();
        }
        int RpcLibClientBase::getClientVersion() const
        {
            return 3; //sync with Python client
        }
        int RpcLibClientBase::getMinRequiredServerVersion() const
        {
            return 3; //sync with Python client
        }
        int RpcLibClientBase::getMinRequiredClientVersion() const
        {
            return pimpl_->client.call("getMinRequiredClientVersion").as<int>();
        }
        int RpcLibClientBase::getServerVersion() const
        {
            return pimpl_->client.call("getServerVersion").as<int>();
        }

        void RpcLibClientBase::reset()
        {
            pimpl_->client.call("reset");
        }

        void RpcLibClientBase::confirmConnection()
        {
            ClockBase* clock = ClockFactory::get();

            // make sure we can talk to the DroneServer
            //std::cout << "Contacting DroneServer..." << std::flush;
            //command_context.client.ping();
            //std::cout << "DroneServer is responding." << std::endl;

            std::cout << "Waiting for connection - " << std::flush;
            const TTimeDelta pause_time = 1;
            while (getConnectionState() != RpcLibClientBase::ConnectionState::Connected) {
                std::cout << "X" << std::flush;
                clock->sleep_for(pause_time);
            }
            std::cout << std::endl
                      << "Connected!" << std::endl;

            auto server_ver = getServerVersion();
            auto client_ver = getClientVersion();
            auto server_min_ver = getMinRequiredServerVersion();
            auto client_min_ver = getMinRequiredClientVersion();

            std::string ver_info = Utils::stringf("Client Ver:%i (Min Req:%i), Server Ver:%i (Min Req:%i)",
                                                  client_ver,
                                                  client_min_ver,
                                                  server_ver,
                                                  server_min_ver);

            if (server_ver < server_min_ver) {
                std::cerr << std::endl
                          << ver_info << std::endl;
                std::cerr << std::endl
                          << "AirSim server is of older version and not supported by this client. Please upgrade!" << std::endl;
            }
            else if (client_ver < client_min_ver) {
                std::cerr << std::endl
                          << ver_info << std::endl;
                std::cerr << std::endl
                          << "AirSim client is of older version and not supported by this server. Please upgrade!" << std::endl;
            }
        }

        bool RpcLibClientBase::armDisarm(bool arm, const std::string& vehicle_name)
        {
            return pimpl_->client.call("armDisarm", arm, vehicle_name).as<bool>();
        }

        msr::airlib::GeoPoint RpcLibClientBase::getHomeGeoPoint(const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getHomeGeoPoint", vehicle_name).as<RpcLibAdaptorsBase::GeoPoint>().to();
        }

        msr::airlib::LidarData RpcLibClientBase::getLidarData(const std::string& lidar_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getLidarData", lidar_name, vehicle_name).as<RpcLibAdaptorsBase::LidarData>().to();
        }

        msr::airlib::ImuBase::Output RpcLibClientBase::getImuData(const std::string& imu_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getImuData", imu_name, vehicle_name).as<RpcLibAdaptorsBase::ImuData>().to();
        }

        msr::airlib::GPULidarData RpcLibClientBase::getGPULidarData(const std::string& lidar_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getGPULidarData", lidar_name, vehicle_name).as<RpcLibAdaptorsBase::GPULidarData>().to();
        }

        msr::airlib::EchoData RpcLibClientBase::getEchoData(const std::string& echo_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getEchoData", echo_name, vehicle_name).as<RpcLibAdaptorsBase::EchoData>().to();
        }

        msr::airlib::SensorTemplateData RpcLibClientBase::getSensorTemplateData(const std::string& echo_name, const std::string& vehicle_name) const
        {
            return msr::airlib::SensorTemplateData();
        }

        msr::airlib::MarLocUwbSensorData RpcLibClientBase::getMarLocUwbSensorData(const std::string& echo_name, const std::string& vehicle_name) const
        {
            return msr::airlib::MarLocUwbSensorData();
        }

        msr::airlib::WifiSensorData RpcLibClientBase::getWifiSensorData(const std::string& echo_name, const std::string& vehicle_name) const
        {
            return msr::airlib::WifiSensorData();
        }

        msr::airlib::BarometerBase::Output RpcLibClientBase::getBarometerData(const std::string& barometer_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getBarometerData", barometer_name, vehicle_name).as<RpcLibAdaptorsBase::BarometerData>().to();
        }

        msr::airlib::MagnetometerBase::Output RpcLibClientBase::getMagnetometerData(const std::string& magnetometer_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getMagnetometerData", magnetometer_name, vehicle_name).as<RpcLibAdaptorsBase::MagnetometerData>().to();
        }

        msr::airlib::GpsBase::Output RpcLibClientBase::getGpsData(const std::string& gps_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getGpsData", gps_name, vehicle_name).as<RpcLibAdaptorsBase::GpsData>().to();
        }

        msr::airlib::DistanceSensorData RpcLibClientBase::getDistanceSensorData(const std::string& distance_sensor_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("getDistanceSensorData", distance_sensor_name, vehicle_name).as<RpcLibAdaptorsBase::DistanceSensorData>().to();
        }

        bool RpcLibClientBase::simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
        {
            return pimpl_->client.call("simSetSegmentationObjectID", mesh_name, object_id, is_name_regex).as<bool>();
        }
        int RpcLibClientBase::simGetSegmentationObjectID(const std::string& mesh_name) const
        {
            return pimpl_->client.call("simGetSegmentationObjectID", mesh_name).as<int>();
        }

        vector<string> RpcLibClientBase::simListAnnotationObjects(const std::string& annotation_name) const
        {
            return pimpl_->client.call("simListAnnotationObjects", annotation_name).as<vector<string>>();
        }

        vector<msr::airlib::Pose> RpcLibClientBase::simListAnnotationPoses(const std::string& annotation_name, bool ned, bool only_visible) const
        {
            const auto& response_adaptor = pimpl_->client.call("simListAnnotationPoses", annotation_name, ned, only_visible).as<vector<RpcLibAdaptorsBase::Pose>>();
            return RpcLibAdaptorsBase::Pose::to(response_adaptor);
        }

        bool RpcLibClientBase::simSetAnnotationObjectID(const std::string& annotation_name, const std::string& mesh_name, int object_id, bool is_name_regex)
        {
            return pimpl_->client.call("simSetAnnotationObjectID", annotation_name, mesh_name, object_id, is_name_regex).as<bool>();
        }

        int RpcLibClientBase::simGetAnnotationObjectID(const std::string& annotation_name, const std::string& mesh_name) const
        {
            return pimpl_->client.call("simGetAnnotationObjectID", annotation_name, mesh_name).as<int>();
        }

        bool RpcLibClientBase::simSetAnnotationObjectColor(const std::string& annotation_name, const std::string& mesh_name, int r, int g, int b, bool is_name_regex)
        {
            return pimpl_->client.call("simSetAnnotationObjectColor", annotation_name, mesh_name, r, g, b, is_name_regex).as<bool>();
        }

        std::string RpcLibClientBase::simGetAnnotationObjectColor(const std::string& annotation_name, const std::string& mesh_name) const
        {
            return pimpl_->client.call("simGetAnnotationObjectColor", annotation_name, mesh_name).as<std::string>();
        }

        bool RpcLibClientBase::simSetAnnotationObjectValue(const std::string& annotation_name, const std::string& mesh_name, float greyscale_value, bool is_name_regex)
        {
            return pimpl_->client.call("simSetAnnotationObjectValue", annotation_name, mesh_name, greyscale_value, is_name_regex).as<bool>();
        }

        float RpcLibClientBase::simGetAnnotationObjectValue(const std::string& annotation_name, const std::string& mesh_name) const
        {
            return pimpl_->client.call("simGetAnnotationObjectValue", annotation_name, mesh_name).as<float>();
        }

        bool RpcLibClientBase::simSetAnnotationObjectTextureByPath(const std::string& annotation_name, const std::string& mesh_name, const std::string& texture_path, bool is_name_regex)
        {
            return pimpl_->client.call("simSetAnnotationObjectTextureByPath", annotation_name, mesh_name, texture_path, is_name_regex).as<bool>();
        }

        bool RpcLibClientBase::simEnableAnnotationObjectTextureByPath(const std::string& annotation_name, const std::string& mesh_name, bool is_name_regex)
        {
            return pimpl_->client.call("simEnableAnnotationObjectTextureByPath", annotation_name, mesh_name, is_name_regex).as<bool>();
        }

        std::string RpcLibClientBase::simGetAnnotationObjectTexturePath(const std::string& annotation_name, const std::string& mesh_name) const
        {
            return pimpl_->client.call("simGetAnnotationObjectTexturePath", annotation_name, mesh_name).as<std::string>();
        }

        void RpcLibClientBase::simAddDetectionFilterMeshName(const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& mesh_name, const std::string& vehicle_name, const std::string& annotation_name)
        {
            pimpl_->client.call("simAddDetectionFilterMeshName", camera_name, type, mesh_name, vehicle_name, annotation_name);
        }
        void RpcLibClientBase::simSetDetectionFilterRadius(const std::string& camera_name, ImageCaptureBase::ImageType type, const float radius_cm, const std::string& vehicle_name, const std::string& annotation_name)
        {
            pimpl_->client.call("simSetDetectionFilterRadius", camera_name, type, radius_cm, vehicle_name, annotation_name);
        }
        void RpcLibClientBase::simClearDetectionMeshNames(const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name, const std::string& annotation_name)
        {
            pimpl_->client.call("simClearDetectionMeshNames", camera_name, type, vehicle_name, annotation_name);
        }
        vector<DetectionInfo> RpcLibClientBase::simGetDetections(const std::string& camera_name, ImageCaptureBase::ImageType image_type, const std::string& vehicle_name, const std::string& annotation_name)
        {
            const auto& result = pimpl_->client.call("simGetDetections", camera_name, image_type, vehicle_name, annotation_name).as<vector<RpcLibAdaptorsBase::DetectionInfo>>();
            return RpcLibAdaptorsBase::DetectionInfo::to(result);
        }

        CollisionInfo RpcLibClientBase::simGetCollisionInfo(const std::string& vehicle_name) const
        {
            return pimpl_->client.call("simGetCollisionInfo", vehicle_name).as<RpcLibAdaptorsBase::CollisionInfo>().to();
        }

        //sim only
        Pose RpcLibClientBase::simGetVehiclePose(const std::string& vehicle_name) const
        {
            return pimpl_->client.call("simGetVehiclePose", vehicle_name).as<RpcLibAdaptorsBase::Pose>().to();
        }
        void RpcLibClientBase::simSetVehiclePose(const Pose& pose, bool ignore_collision, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetVehiclePose", RpcLibAdaptorsBase::Pose(pose), ignore_collision, vehicle_name);
        }

        void RpcLibClientBase::simSetKinematics(const Kinematics::State& state, bool ignore_collision, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetKinematics", RpcLibAdaptorsBase::KinematicsState(state), ignore_collision, vehicle_name);
        }

        void RpcLibClientBase::simSetTraceLine(const std::vector<float>& color_rgba, float thickness, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetTraceLine", color_rgba, thickness, vehicle_name);
        }

        vector<ImageCaptureBase::ImageResponse> RpcLibClientBase::simGetImages(vector<ImageCaptureBase::ImageRequest> request, const std::string& vehicle_name)
        {
            const auto& response_adaptor = pimpl_->client.call("simGetImages",
                                                               RpcLibAdaptorsBase::ImageRequest::from(request),
                                                               vehicle_name)
                                               .as<vector<RpcLibAdaptorsBase::ImageResponse>>();

            return RpcLibAdaptorsBase::ImageResponse::to(response_adaptor);
        }
        vector<uint8_t> RpcLibClientBase::simGetImage(const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name, const std::string& annotation_name)
        {
            vector<uint8_t> result = pimpl_->client.call("simGetImage", camera_name, type, vehicle_name, annotation_name).as<vector<uint8_t>>();
            return result;
        }

        //CinemAirSim
        std::vector<std::string> RpcLibClientBase::simGetPresetLensSettings(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetPresetLensSettings", camera_name, vehicle_name).as<vector<std::string>>();
        }

        std::string RpcLibClientBase::simGetLensSettings(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetLensSettings", camera_name, vehicle_name).as<std::string>();
        }

        void RpcLibClientBase::simSetPresetLensSettings(const std::string& preset_lens_settings, const std::string& camera_name, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetPresetLensSettings", preset_lens_settings, camera_name, vehicle_name);
        }

        std::vector<std::string> RpcLibClientBase::simGetPresetFilmbackSettings(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetPresetFilmbackSettings", camera_name, vehicle_name).as<vector<std::string>>();
        }

        void RpcLibClientBase::simSetPresetFilmbackSettings(const std::string& preset_filmback_settings, const std::string& camera_name, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetPresetFilmbackSettings", preset_filmback_settings, camera_name, vehicle_name);
        }

        std::string RpcLibClientBase::simGetFilmbackSettings(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetFilmbackSettings", camera_name, vehicle_name).as<std::string>();
        }

        float RpcLibClientBase::simSetFilmbackSettings(const float sensor_width, const float sensor_height, const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simSetFilmbackSettings", sensor_width, sensor_height, camera_name, vehicle_name).as<float>();
        }

        float RpcLibClientBase::simGetFocalLength(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetFocalLength", camera_name, vehicle_name).as<float>();
        }

        void RpcLibClientBase::simSetFocalLength(const float focal_length, const std::string& camera_name, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetFocalLength", focal_length, camera_name, vehicle_name);
        }

        void RpcLibClientBase::simEnableManualFocus(const bool enable, const std::string& camera_name, const std::string& vehicle_name)
        {
            pimpl_->client.call("simEnableManualFocus", enable, camera_name, vehicle_name);
        }

        float RpcLibClientBase::simGetFocusDistance(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetFocusDistance", camera_name, vehicle_name).as<float>();
        }
        void RpcLibClientBase::simSetFocusDistance(const float focus_distance, const std::string& camera_name, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetFocusDistance", focus_distance, camera_name, vehicle_name);
        }

        float RpcLibClientBase::simGetFocusAperture(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetFocusAperture", camera_name, vehicle_name).as<float>();
        }

        void RpcLibClientBase::simSetFocusAperture(const float focus_aperture, const std::string& camera_name, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetFocusAperture", focus_aperture, camera_name, vehicle_name);
        }

        void RpcLibClientBase::simEnableFocusPlane(const bool enable, const std::string& camera_name, const std::string& vehicle_name)
        {
            pimpl_->client.call("simEnableFocusPlane", enable, camera_name, vehicle_name);
        }

        std::string RpcLibClientBase::simGetCurrentFieldOfView(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetCurrentFieldOfView", camera_name, vehicle_name).as<std::string>();
        }
        //End CinemAirSim

        // Minor TODO: consider msgpack magic for GeoPoint, so we can have one arg instead of three
        bool RpcLibClientBase::simTestLineOfSightToPoint(const msr::airlib::GeoPoint& point, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simTestLineOfSightToPoint", RpcLibAdaptorsBase::GeoPoint(point), vehicle_name).as<bool>();
        }

        bool RpcLibClientBase::simTestLineOfSightBetweenPoints(const msr::airlib::GeoPoint& point1, const msr::airlib::GeoPoint& point2)
        {
            return pimpl_->client.call("simTestLineOfSightBetweenPoints", RpcLibAdaptorsBase::GeoPoint(point1), RpcLibAdaptorsBase::GeoPoint(point2)).as<bool>();
        }

        vector<msr::airlib::GeoPoint> RpcLibClientBase::simGetWorldExtents()
        {
            vector<RpcLibAdaptorsBase::GeoPoint> raw_result = pimpl_->client.call("simGetWorldExtents").as<vector<RpcLibAdaptorsBase::GeoPoint>>();
            vector<msr::airlib::GeoPoint> final_result;

            // Convert
            final_result.push_back(raw_result[0].to());
            final_result.push_back(raw_result[1].to());

            return final_result;
        }

        vector<MeshPositionVertexBuffersResponse> RpcLibClientBase::simGetMeshPositionVertexBuffers()
        {
            const auto& response_adaptor = pimpl_->client.call("simGetMeshPositionVertexBuffers").as<vector<RpcLibAdaptorsBase::MeshPositionVertexBuffersResponse>>();
            return RpcLibAdaptorsBase::MeshPositionVertexBuffersResponse::to(response_adaptor);
        }

        bool RpcLibClientBase::simAddVehicle(const std::string& vehicle_name, const std::string& vehicle_type, const Pose& pose, const std::string& pawn_path)
        {
            return pimpl_->client.call("simAddVehicle", vehicle_name, vehicle_type, RpcLibAdaptorsBase::Pose(pose), pawn_path).as<bool>();
        }

		bool RpcLibClientBase::simAddObstacle(const Pose& pose, float speed, const std::string& obstacle_name)
		{
			return pimpl_->client.call("simAddObstacle", RpcLibAdaptorsBase::Pose(pose),speed, obstacle_name).as<bool>();
		}

		bool RpcLibClientBase::activateGeneration(bool landscape)
		{
			return pimpl_->client.call("activateGeneration", landscape).as<bool>();
		}

		bool RpcLibClientBase::generatePortTerrain(const std::string& type, int seed, int length, float mina, float maxa, float mind, float maxd)
		{
			return pimpl_->client.call("generatePortTerrain", type, seed, length, mina, maxa, mind, maxd).as<bool>();
		}
        vector<msr::airlib::Vector2r> RpcLibClientBase::getGoal(int distance, const msr::airlib::Vector2r& initial_location) const
        {
            const auto& response_adaptor = pimpl_->client.call("getGoal", distance, RpcLibAdaptorsBase::Vector2r(initial_location)).as<vector<RpcLibAdaptorsBase::Vector2r>>();
			return RpcLibAdaptorsBase::Vector2r::to(response_adaptor);
        }
        vector<msr::airlib::Vector2r> RpcLibClientBase::getRelLocation(const msr::airlib::Vector2r& initial_location) const
        {
            const auto& response_adaptor = pimpl_->client.call("getRelLocation", RpcLibAdaptorsBase::Vector2r(initial_location)).as<vector<RpcLibAdaptorsBase::Vector2r>>();
            return RpcLibAdaptorsBase::Vector2r::to(response_adaptor);
        }
        void RpcLibClientBase::simPrintLogMessage(const std::string& message, std::string message_param, unsigned char severity)
        {
            pimpl_->client.call("simPrintLogMessage", message, message_param, severity);
        }

        void RpcLibClientBase::simFlushPersistentMarkers()
        {
            pimpl_->client.call("simFlushPersistentMarkers");
        }

        void RpcLibClientBase::simPlotPoints(const vector<Vector3r>& points, const vector<float>& color_rgba, float size, float duration, bool is_persistent)
        {
            vector<RpcLibAdaptorsBase::Vector3r> conv_points;
            RpcLibAdaptorsBase::from(points, conv_points);
            pimpl_->client.call("simPlotPoints", conv_points, color_rgba, size, duration, is_persistent);
        }

        void RpcLibClientBase::simPlotLineStrip(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
        {
            vector<RpcLibAdaptorsBase::Vector3r> conv_points;
            RpcLibAdaptorsBase::from(points, conv_points);
            pimpl_->client.call("simPlotLineStrip", conv_points, color_rgba, thickness, duration, is_persistent);
        }

        void RpcLibClientBase::simPlotLineList(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
        {
            vector<RpcLibAdaptorsBase::Vector3r> conv_points;
            RpcLibAdaptorsBase::from(points, conv_points);
            pimpl_->client.call("simPlotLineList", conv_points, color_rgba, thickness, duration, is_persistent);
        }

        void RpcLibClientBase::simPlotArrows(const vector<Vector3r>& points_start, const vector<Vector3r>& points_end, const vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent)
        {
            vector<RpcLibAdaptorsBase::Vector3r> conv_points_start;
            RpcLibAdaptorsBase::from(points_start, conv_points_start);
            vector<RpcLibAdaptorsBase::Vector3r> conv_points_end;
            RpcLibAdaptorsBase::from(points_end, conv_points_end);
            pimpl_->client.call("simPlotArrows", conv_points_start, conv_points_end, color_rgba, thickness, arrow_size, duration, is_persistent);
        }

        void RpcLibClientBase::simPlotStrings(const vector<std::string>& strings, const vector<Vector3r>& positions, float scale, const vector<float>& color_rgba, float duration)
        {
            vector<RpcLibAdaptorsBase::Vector3r> conv_positions;
            RpcLibAdaptorsBase::from(positions, conv_positions);
            pimpl_->client.call("simPlotStrings", strings, conv_positions, scale, color_rgba, duration);
        }

        void RpcLibClientBase::simPlotTransforms(const vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent)
        {
            vector<RpcLibAdaptorsBase::Pose> conv_poses;
            RpcLibAdaptorsBase::from(poses, conv_poses);
            pimpl_->client.call("simPlotTransforms", conv_poses, scale, thickness, duration, is_persistent);
        }

        void RpcLibClientBase::simPlotTransformsWithNames(const vector<Pose>& poses, const vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const vector<float>& text_color_rgba, float duration)
        {
            vector<RpcLibAdaptorsBase::Pose> conv_poses;
            RpcLibAdaptorsBase::from(poses, conv_poses);
            pimpl_->client.call("simPlotTransformsWithNames", conv_poses, names, tf_scale, tf_thickness, text_scale, text_color_rgba, duration);
        }

        bool RpcLibClientBase::simIsPaused() const
        {
            return pimpl_->client.call("simIsPaused").as<bool>();
        }

        void RpcLibClientBase::simPause(bool is_paused)
        {
            pimpl_->client.call("simPause", is_paused);
        }

        void RpcLibClientBase::simContinueForTime(double seconds)
        {
            pimpl_->client.call("simContinueForTime", seconds);
        }

        void RpcLibClientBase::simContinueForFrames(uint32_t frames)
        {
            pimpl_->client.call("simContinueForFrames", frames);
        }

        void RpcLibClientBase::simEnableWeather(bool enable)
        {
            pimpl_->client.call("simEnableWeather", enable);
        }
        void RpcLibClientBase::simSetWeatherParameter(WorldSimApiBase::WeatherParameter param, float val)
        {
            pimpl_->client.call("simSetWeatherParameter", param, val);
        }

        void RpcLibClientBase::simSetTimeOfDay(bool is_enabled, const string& start_datetime, bool is_start_datetime_dst,
                                               float celestial_clock_speed, float update_interval_secs, bool move_sun)
        {
            pimpl_->client.call("simSetTimeOfDay", is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed, update_interval_secs, move_sun);
        }

        vector<string> RpcLibClientBase::simListSceneObjects(const string& name_regex) const
        {
            return pimpl_->client.call("simListSceneObjects", name_regex).as<vector<string>>();
        }

        vector<string> RpcLibClientBase::simListInstanceSegmentationObjects() const
        {
            return pimpl_->client.call("simListInstanceSegmentationObjects").as<vector<string>>();
        }

        vector<msr::airlib::Vector3r> RpcLibClientBase::simGetInstanceSegmentationColorMap() const
        {
            const auto& response_adaptor = pimpl_->client.call("simGetInstanceSegmentationColorMap").as<vector<RpcLibAdaptorsBase::Vector3r>>();
            return RpcLibAdaptorsBase::Vector3r::to(response_adaptor);
        }

        vector<msr::airlib::Pose> RpcLibClientBase::simListInstanceSegmentationPoses(bool ned, bool only_visible) const
        {
            const auto& response_adaptor = pimpl_->client.call("simListInstanceSegmentationPoses", ned, only_visible).as<vector<RpcLibAdaptorsBase::Pose>>();
            return RpcLibAdaptorsBase::Pose::to(response_adaptor);
        }

        std::vector<std::string> RpcLibClientBase::simSwapTextures(const std::string& tags, int tex_id, int component_id, int material_id)
        {
            return pimpl_->client.call("simSwapTextures", tags, tex_id, component_id, material_id).as<vector<string>>();
        }

        bool RpcLibClientBase::simSetObjectMaterial(const std::string& object_name, const std::string& material_name, const int component_id)
        {
            return pimpl_->client.call("simSetObjectMaterial", object_name, material_name, component_id).as<bool>();
        }

        bool RpcLibClientBase::simSetObjectMaterialFromTexture(const std::string& object_name, const std::string& texture_path, const int component_id)
        {
            return pimpl_->client.call("simSetObjectMaterialFromTexture", object_name, texture_path, component_id).as<bool>();
        }

        bool RpcLibClientBase::simLoadLevel(const string& level_name)
        {
            return pimpl_->client.call("simLoadLevel", level_name).as<bool>();
        }

        std::string RpcLibClientBase::simSpawnObject(const std::string& object_name, const std::string& load_component, const Pose& pose,
                                                     const Vector3r& scale, bool physics_enabled)
        {
            return pimpl_->client.call("simSpawnObject", object_name, load_component, RpcLibAdaptorsBase::Pose(pose), RpcLibAdaptorsBase::Vector3r(scale), physics_enabled).as<std::string>();
        }

        bool RpcLibClientBase::simDestroyObject(const std::string& object_name)
        {
            return pimpl_->client.call("simDestroyObject", object_name).as<bool>();
        }

        msr::airlib::Vector3r RpcLibClientBase::simGetObjectScale(const std::string& object_name) const
        {
            return pimpl_->client.call("simGetObjectScale", object_name).as<RpcLibAdaptorsBase::Vector3r>().to();
        }

        msr::airlib::Pose RpcLibClientBase::simGetObjectPose(const std::string& object_name, bool ned) const
        {
            return pimpl_->client.call("simGetObjectPose", object_name, ned).as<RpcLibAdaptorsBase::Pose>().to();
        }

        bool RpcLibClientBase::simSetObjectPose(const std::string& object_name, const msr::airlib::Pose& pose, bool teleport)
        {
            return pimpl_->client.call("simSetObjectPose", object_name, RpcLibAdaptorsBase::Pose(pose), teleport).as<bool>();
        }

        bool RpcLibClientBase::simSetObjectScale(const std::string& object_name, const msr::airlib::Vector3r& scale)
        {
            return pimpl_->client.call("simSetObjectScale", object_name, RpcLibAdaptorsBase::Vector3r(scale)).as<bool>();
        }

        CameraInfo RpcLibClientBase::simGetCameraInfo(const std::string& camera_name, const std::string& vehicle_name) const
        {
            return pimpl_->client.call("simGetCameraInfo", camera_name, vehicle_name).as<RpcLibAdaptorsBase::CameraInfo>().to();
        }

        void RpcLibClientBase::simSetCameraPose(const std::string& camera_name, const Pose& pose, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetCameraPose", camera_name, RpcLibAdaptorsBase::Pose(pose), vehicle_name);
        }

        void RpcLibClientBase::simSetCameraFov(const std::string& camera_name, float fov_degrees, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetCameraFov", camera_name, fov_degrees, vehicle_name);
        }

        void RpcLibClientBase::simSetDistortionParam(const std::string& camera_name, const std::string& param_name, float value, const std::string& vehicle_name)
        {
            pimpl_->client.call("simSetDistortionParam", camera_name, param_name, value, vehicle_name);
        }

        std::vector<float> RpcLibClientBase::simGetDistortionParams(const std::string& camera_name, const std::string& vehicle_name)
        {
            return pimpl_->client.call("simGetDistortionParams", camera_name, vehicle_name).as<std::vector<float>>();
        }

        msr::airlib::Kinematics::State RpcLibClientBase::simGetGroundTruthKinematics(const std::string& vehicle_name) const
        {
            return pimpl_->client.call("simGetGroundTruthKinematics", vehicle_name).as<RpcLibAdaptorsBase::KinematicsState>().to();
        }
        msr::airlib::Environment::State RpcLibClientBase::simGetGroundTruthEnvironment(const std::string& vehicle_name) const
        {
            return pimpl_->client.call("simGetGroundTruthEnvironment", vehicle_name).as<RpcLibAdaptorsBase::EnvironmentState>().to();
            ;
        }
        bool RpcLibClientBase::simCreateVoxelGrid(const msr::airlib::Vector3r& position, const int& x, const int& y, const int& z, const float& res, const std::string& output_file)
        {
            return pimpl_->client.call("simCreateVoxelGrid", RpcLibAdaptorsBase::Vector3r(position), x, y, z, res, output_file).as<bool>();
        }

        void RpcLibClientBase::cancelLastTask(const std::string& vehicle_name)
        {
            pimpl_->client.call("cancelLastTask", vehicle_name);
        }

        bool RpcLibClientBase::simRunConsoleCommand(const std::string& command)
        {
            return pimpl_->client.call("simRunConsoleCommand", command).as<bool>();
        }

        //return value of last task. It should be true if task completed without
        //cancellation or timeout
        RpcLibClientBase* RpcLibClientBase::waitOnLastTask(bool* task_result, float timeout_sec)
        {
            //should be implemented by derived class if it supports async task,
            //for example using futures
            unused(timeout_sec);
            unused(task_result);

            return this;
        }

        void RpcLibClientBase::startRecording()
        {
            pimpl_->client.call("startRecording");
        }

        void RpcLibClientBase::stopRecording()
        {
            pimpl_->client.call("stopRecording");
        }

        bool RpcLibClientBase::isRecording()
        {
            return pimpl_->client.call("isRecording").as<bool>();
        }

        void RpcLibClientBase::simSetWind(const Vector3r& wind) const
        {
            RpcLibAdaptorsBase::Vector3r conv_wind(wind);
            pimpl_->client.call("simSetWind", conv_wind);
        }
        void RpcLibClientBase::simSetExtForce(const Vector3r& ext_force) const
        {
            RpcLibAdaptorsBase::Vector3r conv_ext_force(ext_force);
            pimpl_->client.call("simSetExtForce", conv_ext_force);
        }
        vector<string> RpcLibClientBase::listVehicles()
        {
            return pimpl_->client.call("listVehicles").as<vector<string>>();
        }

        std::string RpcLibClientBase::getSettingsString() const
        {
            return pimpl_->client.call("getSettingsString").as<std::string>();
        }

        std::vector<std::string> RpcLibClientBase::simListAssets() const
        {
            return pimpl_->client.call("simListAssets").as<std::vector<std::string>>();
        }

        void* RpcLibClientBase::getClient()
        {
            return &pimpl_->client;
        }
        const void* RpcLibClientBase::getClient() const
        {
            return &pimpl_->client;
        }
    }
} //namespace
#endif
#endif
