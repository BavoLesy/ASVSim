// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef air_VesselApiBase_hpp
#define air_VesselApiBase_hpp

#include <array>
#include <stdexcept>
#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"

namespace msr {
    namespace airlib {

		enum DisturbanceType {
			Wind,
			Wave,
			Current
		};

        class VesselApiBase : public VehicleApiBase {
        public:
            
            /*virtual void enableApiControl(bool is_enabled) = 0;
            virtual bool isApiControlEnabled() const = 0;
            virtual bool armDisarm(bool arm) = 0;
            virtual GeoPoint getHomeGeoPoint() const = 0;*/

			constexpr static unsigned int MAX_THRUSTER_COUNT = 10;

            struct VesselControls {
				std::array<float, MAX_THRUSTER_COUNT> thruster_forces;
				std::array<float, MAX_THRUSTER_COUNT> thruster_angles;

                VesselControls()
                {
                    thruster_forces.fill(0.0);
                    thruster_angles.fill(0.5);
                }
                VesselControls(std::array<float, MAX_THRUSTER_COUNT> thruster_forces, std::array<float, MAX_THRUSTER_COUNT> thruster_angles)
                    : thruster_forces(thruster_forces), thruster_angles(thruster_angles)
                {
                }
                void set_thrust(float thrust_val, unsigned int idx)
                {
					if (idx >= MAX_THRUSTER_COUNT) {
						throw std::out_of_range("Thruster index out of (compile-time) range."
                            "If you need more than 10 thrusters, modify 'MAX_THRUSTER_COUNT' in VesselApiBase.hpp");
					}
                    thruster_forces[idx] = thrust_val;
                }

                void set_angle (float angle_val, unsigned int idx)
                {
                    if (idx >= MAX_THRUSTER_COUNT) {
                        throw std::out_of_range("Thruster index out of (compile-time) range."
                            "If you need more than 10 thrusters, modify 'MAX_THRUSTER_COUNT' in VesselApiBase.hpp");
                    }
                    thruster_angles[idx] = angle_val;
                }
            };

            struct DisturbanceControls {
                float wind_force = 0.0;         // N
				float wind_angle = 0.0;         // radians
				//float wave_force = 0.0;       // N
				//float wave_angle = 0.0;       // radians
				float current_velocity = 0.0;      // N
				float current_angle = 0.0;      // radians

                DisturbanceControls()
                {
                }
				DisturbanceControls(float wind_force, float wind_angle, float current_velocity, float current_angle)
					: wind_force(wind_force), wind_angle(wind_angle), current_velocity(current_velocity), current_angle(current_angle)
                {
                }
				void set_wind(float wind_force_val, float wind_angle_val)
				{
					wind_force = wind_force_val;
					wind_angle = wind_angle_val;
				}
				//void set_wave(float wave_force_val, float wave_angle_val)
				//{
				//	wave_force = wave_force_val;
				//	wave_angle = wave_angle_val;
				//}
                void set_current(float current_velocity_val, float current_angle_val)
                {
                    current_velocity = current_velocity_val;
					current_angle = current_angle_val;
                }
            };

            struct VesselState {
                Kinematics::State kinematics_estimated;
                uint64_t timestamp;
                
            public:
                VesselState(const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
                    : kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
                {
                }

                VesselState() = default;
            };

        public:

            // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
            //CarApiBase() {}

            VesselApiBase(const AirSimSettings::VehicleSetting* vehicle_setting,
                std::shared_ptr<SensorFactory> sensor_factory,
                const Kinematics::State& state, const Environment& environment)
            {
                initialize(vehicle_setting, sensor_factory, state, environment);
            }

            virtual real_T getActuation(unsigned int actuation_index) const override
            {
                real_T control_signal;
				unsigned int thruster_index = actuation_index / 2; //each thruster has two actuation signals: thrust and angle
                if (actuation_index % 2 == 0)
                    control_signal = last_controls_.thruster_forces[thruster_index];
                else
                    control_signal = last_controls_.thruster_angles[thruster_index];

                return control_signal;
            }

            virtual std::pair<real_T, real_T> getDisturbanceAngle(DisturbanceType disturbance_type) const
			{
				real_T force, angle;
				switch (disturbance_type)
				{
				case DisturbanceType::Wind:
					force = last_disturbances_.wind_force;
					angle = last_disturbances_.wind_angle;
					break;
				//case DisturbanceType::Wave:
				//	force = last_disturbances_.wave_force;
				//	angle = last_disturbances_.wave_angle;
				//	break;
				case DisturbanceType::Current:
					force = last_disturbances_.current_velocity;
					angle = last_disturbances_.current_angle;
					break;
				default:
					force = 0.0;
					angle = 0.0;
					break;
				}
				return std::make_pair(force, angle);
			}

            virtual Vector2r getDisturbanceXY(DisturbanceType disturbance_type) const
            {
                real_T force, angle;
                switch (disturbance_type)
                {
                case DisturbanceType::Wind:
                    force = last_disturbances_.wind_force;
                    angle = last_disturbances_.wind_angle;
                    break;
                //case DisturbanceType::Wave:
                //    force = last_disturbances_.wave_force;
                //    angle = last_disturbances_.wave_angle;
                //    break;
                case DisturbanceType::Current:
                    force = last_disturbances_.current_velocity;
                    angle = last_disturbances_.current_angle;
                    break;
                default:
                    force = 0.0;
                    angle = 0.0;
                    break;
                }
                return Vector2r(force * std::cos(angle), force * std::sin(angle));
            }

            //default implementation so derived class doesn't have to call on VehicleApiBase
            virtual void resetImplementation() override
            {
                token_.cancel();
                VehicleApiBase::resetImplementation();
                //reset sensors last after their ground truth has been reset
                getSensors().reset();
            }
            virtual void update(float delta = 0) override
            {
                VehicleApiBase::update(delta);

                getSensors().update(delta);
            }
            void reportState(StateReporter& reporter) override
            {
                getSensors().reportState(reporter);
            }

            // sensor helpers
            virtual const SensorCollection& getSensors() const override
            {
                return sensors_;
            }

            SensorCollection& getSensors()
            {
                return sensors_;
            }

            void initialize(const AirSimSettings::VehicleSetting* vehicle_setting,
                std::shared_ptr<SensorFactory> sensor_factory,
                const Kinematics::State& state, const Environment& environment)
            {
                sensor_factory_ = sensor_factory;

                sensor_storage_.clear();
                sensors_.clear();

                addSensorsFromSettings(vehicle_setting);

                getSensors().initialize(&state, &environment);
            }

            void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
            {
                // use sensors from vehicle settings; if empty list, use default sensors.
                // note that the vehicle settings completely override the default sensor "list";
                // there is no piecemeal add/remove/update per sensor.
                //const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>>& sensor_settings
                //    = vehicle_setting->sensors.size() > 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;
                const auto& sensor_settings = vehicle_setting->sensors;

                sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
            }

            virtual void setVesselControls(const VesselControls& controls) {
                last_controls_ = controls;
            }

			virtual void setDisturbanceControls(const DisturbanceControls& controls) {
				last_disturbances_ = controls;
			}
            // virtual VesselState getVesselState() const;
            virtual const VesselApiBase::VesselControls& getVesselControls() const {
                return last_controls_;
            }

			virtual const VesselApiBase::DisturbanceControls& getDisturbances() const {
				return last_disturbances_;
			}

            virtual VesselState getVesselState() const
            {
                VesselState state;
                state.kinematics_estimated = Kinematics::State();
                state.timestamp = clock()->nowNanos();
                return state;
            }

            virtual ~VesselApiBase() = default;

            std::shared_ptr<const SensorFactory> sensor_factory_;
            SensorCollection sensors_; //maintains sensor type indexed collection of sensors
            vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
            VesselControls last_controls_;
			DisturbanceControls last_disturbances_;
        
        private:
            CancelToken token_;
        };
       

    }
} //namespace

#endif