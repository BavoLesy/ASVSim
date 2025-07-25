// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_WorldSimApiBase_hpp
#define air_WorldSimApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"

namespace msr
{
namespace airlib
{

    class WorldSimApiBase
    {
    public:
        enum class WeatherParameter
        {
            Rain = 0,
            Roadwetness = 1,
            Snow = 2,
            RoadSnow = 3,
            MapleLeaf = 4,
            RoadLeaf = 5,
            Dust = 6,
            Fog = 7,
            Enabled = 8
        };

        virtual ~WorldSimApiBase() = default;

        // ------ Level setting apis ----- //
        virtual bool loadLevel(const std::string& level_name) = 0;
        virtual string spawnObject(const std::string& object_name, const std::string& load_component, const Pose& pose, const Vector3r& scale, bool physics_enabled, bool is_blueprint) = 0;
        virtual bool destroyObject(const std::string& object_name) = 0;
        virtual std::vector<std::string> listAssets() const = 0;

        virtual bool isPaused() const = 0;
        virtual void reset() = 0;
        virtual void pause(bool is_paused) = 0;
        virtual void continueForTime(double seconds) = 0;
        virtual void continueForFrames(uint32_t frames) = 0;

        virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
                                  float celestial_clock_speed, float update_interval_secs, bool move_sun) = 0;

        virtual void enableWeather(bool enable) = 0;
        virtual void setWeatherParameter(WeatherParameter param, float val) = 0;

        virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;
        virtual int getSegmentationObjectID(const std::string& mesh_name) const = 0;

        virtual std::vector<std::string> listAnnotationObjects(const std::string& annotation_name) const = 0;
        virtual std::vector<Pose> listAnnotationPoses(const std::string& annotation_name, bool ned = true, bool only_visible = false) const = 0;

        virtual bool setAnnotationObjectID(const std::string& annotation_name, const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;
        virtual int getAnnotationObjectID(const std::string& annotation_name, const std::string& mesh_name) const = 0;
        virtual bool setAnnotationObjectColor(const std::string& annotation_name, const std::string& mesh_name, int r, int g, int b, bool is_name_regex = false) = 0;
        virtual std::string getAnnotationObjectColor(const std::string& annotation_name, const std::string& mesh_name) const = 0;
        virtual bool setAnnotationObjectValue(const std::string& annotation_name, const std::string& mesh_name, float greyscale_value, bool is_name_regex = false) = 0;
        virtual float getAnnotationObjectValue(const std::string& annotation_name, const std::string& mesh_name) const = 0;
        virtual bool setAnnotationObjectTextureByPath(const std::string& annotation_name, const std::string& mesh_name, const std::string& texture_path, bool is_name_regex = false) = 0;
        virtual bool enableAnnotationObjectTextureByPath(const std::string& annotation_name, const std::string& mesh_name, bool is_name_regex = false) = 0;
        virtual std::string getAnnotationObjectTexturePath(const std::string& annotation_name, const std::string& mesh_name) const = 0;

        virtual bool addVehicle(const std::string& vehicle_name, const std::string& vehicle_type, const Pose& pose, const std::string& pawn_path = "") = 0;

		virtual bool addObstacle(const Pose& pose, float speed = 1000.0, const std::string& vehicle_name = "") = 0;

		virtual bool activateGeneration(bool landscape = false) = 0;

		virtual bool generatePortTerrain(const std::string& type = "", int seed = -464588337, int length = 10, float mina = -45.0, float maxa = 45.0, float mind = 3000.0, float maxd = 6000.0) = 0;

        virtual std::vector<Vector2r> getGoal(int distance = 12, const Vector2r& initial_location = Vector2r(0, 0)) const = 0;

		virtual std::vector<Vector2r> getRelLocation(const Vector2r& initial_location = Vector2r(0, 0)) const = 0;

        virtual void printLogMessage(const std::string& message,
                                     const std::string& message_param = "", unsigned char severity = 0) = 0;

        //----------- Plotting APIs ----------/
        virtual void simFlushPersistentMarkers() = 0;
        virtual void simPlotPoints(const vector<Vector3r>& points, const vector<float>& color_rgba, float size, float duration, bool is_persistent) = 0;
        virtual void simPlotLineStrip(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent) = 0;
        virtual void simPlotLineList(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent) = 0;
        virtual void simPlotArrows(const vector<Vector3r>& points_start, const vector<Vector3r>& points_end, const vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent) = 0;
        virtual void simPlotStrings(const vector<std::string>& strings, const vector<Vector3r>& positions, float scale, const vector<float>& color_rgba, float duration) = 0;
        virtual void simPlotTransforms(const vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent) = 0;
        virtual void simPlotTransformsWithNames(const vector<Pose>& poses, const vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const vector<float>& text_color_rgba, float duration) = 0;

        virtual std::vector<std::string> listSceneObjects(const std::string& name_regex) const = 0;
        virtual std::vector<std::string> listInstanceSegmentationObjects() const = 0;
        virtual std::vector<Vector3r> getInstanceSegmentationColorMap() const = 0;
        virtual std::vector<Pose> listInstanceSegmentationPoses(bool ned = true, bool only_visible = false) const = 0;
        virtual Pose getObjectPose(const std::string& object_name, bool ned = true) const = 0;
        virtual Vector3r getObjectScale(const std::string& object_name) const = 0;
        virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) = 0;
        virtual bool runConsoleCommand(const std::string& command) = 0;
        virtual bool setObjectScale(const std::string& object_name, const Vector3r& scale) = 0;
        virtual std::unique_ptr<std::vector<std::string>> swapTextures(const std::string& tag, int tex_id = 0, int component_id = 0, int material_id = 0) = 0;
        virtual bool setLightIntensity(const std::string& light_name, float intensity) = 0;
        virtual bool setObjectMaterial(const std::string& object_name, const std::string& material_name, const int component_id = 0) = 0;
        virtual bool setObjectMaterialFromTexture(const std::string& object_name, const std::string& texture_path, const int component_id = 0) = 0;
        virtual vector<MeshPositionVertexBuffersResponse> getMeshPositionVertexBuffers() const = 0;

        virtual bool createVoxelGrid(const Vector3r& position, const int& x_size, const int& y_size, const int& z_size, const float& res, const std::string& output_file) = 0;

        // Recording APIs
        virtual void startRecording() = 0;
        virtual void stopRecording() = 0;
        virtual bool isRecording() const = 0;

        virtual void setWind(const Vector3r& wind) const = 0;
        virtual void setExtForce(const Vector3r& ext_force) const = 0;
        virtual vector<string> listVehicles() const = 0;

        virtual std::string getSettingsString() const = 0;

        virtual bool testLineOfSightBetweenPoints(const msr::airlib::GeoPoint& point1, const msr::airlib::GeoPoint& point2) const = 0;
        virtual vector<msr::airlib::GeoPoint> getWorldExtents() const = 0;

        // Camera APIs
        virtual CameraInfo getCameraInfo(const CameraDetails& camera_details) const = 0;
        virtual void setCameraPose(const msr::airlib::Pose& pose, const CameraDetails& camera_details) = 0;
        virtual void setCameraFoV(float fov_degrees, const CameraDetails& camera_details) = 0;
        virtual void setDistortionParam(const std::string& param_name, float value, const CameraDetails& camera_details) = 0;
        virtual std::vector<float> getDistortionParams(const CameraDetails& camera_details) const = 0;

        virtual std::vector<ImageCaptureBase::ImageResponse> getImages(const std::vector<ImageCaptureBase::ImageRequest>& requests,
                                                                       const std::string& vehicle_name) const = 0;
        virtual std::vector<uint8_t> getImage(ImageCaptureBase::ImageType image_type, const CameraDetails& camera_details, const std::string& annotation_name) const = 0;

        //CinemAirSim
        virtual std::vector<std::string> getPresetLensSettings(const CameraDetails& camera_details) = 0;
        virtual std::string getLensSettings(const CameraDetails& camera_details) = 0;
        virtual void setPresetLensSettings(std::string, const CameraDetails& camera_details) = 0;
        virtual std::vector<std::string> getPresetFilmbackSettings(const CameraDetails& camera_details) = 0;
        virtual void setPresetFilmbackSettings(std::string, const CameraDetails& camera_details) = 0;
        virtual std::string getFilmbackSettings(const CameraDetails& camera_details) = 0;
        virtual float setFilmbackSettings(float width, float height, const CameraDetails& camera_details) = 0;
        virtual float getFocalLength(const CameraDetails& camera_details) = 0;
        virtual void setFocalLength(float focal_length, const CameraDetails& camera_details) = 0;
        virtual void enableManualFocus(bool enable, const CameraDetails& camera_details) = 0;
        virtual float getFocusDistance(const CameraDetails& camera_details) = 0;
        virtual void setFocusDistance(float focus_distance, const CameraDetails& camera_details) = 0;
        virtual float getFocusAperture(const CameraDetails& camera_details) = 0;
        virtual void setFocusAperture(float focus_aperture, const CameraDetails& camera_details) = 0;
        virtual void enableFocusPlane(bool enable, const CameraDetails& camera_details) = 0;
        virtual std::string getCurrentFieldOfView(const CameraDetails& camera_details) = 0;
        //end CinemAirSim

        virtual void addDetectionFilterMeshName(ImageCaptureBase::ImageType image_type, const std::string& mesh_name, const CameraDetails& camera_details, const std::string& annotation_name) = 0;
        virtual void setDetectionFilterRadius(ImageCaptureBase::ImageType image_type, float radius_cm, const CameraDetails& camera_details, const std::string& annotation_name) = 0;
        virtual void clearDetectionMeshNames(ImageCaptureBase::ImageType image_type, const CameraDetails& camera_details, const std::string& annotation_name) = 0;
        virtual std::vector<DetectionInfo> getDetections(ImageCaptureBase::ImageType image_type, const CameraDetails& camera_details, const std::string& annotation_name) = 0;
    };
}
} //namespace

#endif
