// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_VesselParameters_hpp
#define msr_airlib_VesselParameters_hpp

#include "common/Common.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/vessel/api/VesselApiBase.hpp"
#include "ThrusterParams.hpp"
#include <optional>
#include <stdexcept>


namespace msr {
    namespace airlib {

        class VesselParams {
            //All units are SI
        public: //types

            struct Params {
                std::optional<real_T> design_speed = 0.0;
                real_T mass = 0.0;
                real_T center_x = 0.0;        // Center of gravity w.r.t x axys. 
                real_T length = 0.0;
                Matrix3x3r inertia = Matrix3x3r::Zero();

                // X-coefficients
                real_T X_u = 0.0;     // X_u
                real_T X_u_u = 0.0;   // X_uu
                real_T X_uuu = 0.0;   // X_uuu
                real_T X_v_v = 0.0;   // X_vv
                real_T X_r_r = 0.0;   // X_rr
                real_T X_r_v = 0.0;   // X_vr
                real_T X_d_d = 0.0;   // X_dd
                real_T X_v_d = 0.0;   // X_vd
                real_T X_r_d = 0.0;   // X_rd
                real_T X_v_r = 0.0;   // X_vr

                // Y-coefficients
                real_T Y_v = 0.0;     // Y_v
                real_T Y_v_v = 0.0;   // Y_vv
                real_T Y_vvv = 0.0;   // Y_vvv
                real_T Y_r = 0.0;     // Y_r
                real_T Y_v_r = 0.0;   // Y_vr
                real_T Y_r_r = 0.0;   // Y_rr
                real_T Y_rrr = 0.0;   // Y_rrr
                real_T Y_d = 0.0;     // Y_d
                real_T Y_d_d = 0.0;   // Y_dd
                real_T Y_d_d_d = 0.0; // Y_ddd
                real_T Y_v_r_r = 0.0; // Y_vrr
                real_T Y_v_v_r = 0.0; // Y_vvr
                real_T Y_u_d = 0.0;   // Y_ud
                real_T Y_v_d_d = 0.0; // Y_vdd
                real_T Y_v_v_d = 0.0; // Y_vvd
                real_T Y_r_d_d = 0.0; // Y_rdd
                real_T Y_r_r_d = 0.0; // Y_rrd
                real_T Y_r_v = 0.0;

                // N-coefficients
                real_T N_v = 0.0;     // N_v
                real_T N_v_v = 0.0;   // N_vv
                real_T N_vvv = 0.0;   // N_vvv
                real_T N_r = 0.0;     // N_r
                real_T N_r_r = 0.0;   // N_rr
                real_T N_rrr = 0.0;   // N_rrr
                real_T N_d = 0.0;     // N_d
                real_T N_d_d = 0.0;   // N_dd
                real_T N_d_d_d = 0.0; // N_ddd
                real_T N_v_r_r = 0.0; // N_vrr
                real_T N_v_v_r = 0.0; // N_vvr
                real_T N_u_d = 0.0;   // N_ud
                real_T N_v_d_d = 0.0; // N_vdd
                real_T N_v_v_d = 0.0; // N_vvd
                real_T N_r_d_d = 0.0; // N_rdd
                real_T N_r_r_d = 0.0; // N_rrd
                real_T N_r_v = 0.0;
                real_T N_v_r = 0.0;

                // Optional terms from: https://www.joet.org/upload/pdf/KSOE-2023-019.pdf
                real_T Y_0 = 0.0;
                real_T Y_0_u = 0.0;
                real_T Y_0_u_u = 0.0;
                real_T N_0 = 0.0;
                real_T N_0_u = 0.0;
                real_T N_0_u_u = 0.0;

                real_T N_vu = 0.0;
                real_T N_ru = 0.0;
                real_T N_ud = 0.0;
                real_T N_u_u_d = 0.0;
                real_T Y_vu = 0.0;
                real_T Y_ru = 0.0;
                real_T Y_ud = 0.0;
                real_T Y_u_u_d = 0.0;
                real_T X_udd = 0.0;
                real_T X_uvd = 0.0;

                // Dimensionless parameters
                bool dimensionless = false;
				bool large_vessel = false;
                real_T mass_dimensionless = 0.0;
                real_T xg_dimensionless = 0.0;

                // https://doi.org/10.1016/j.oceaneng.2023.115412 : eqn (7)
                Matrix3x3r mass_matrix = Matrix3x3r::Zero();
                Matrix3x3r inverse_mass_matrix = Matrix3x3r::Zero();
                std::vector<ThrusterParams> rudder_params;
            };

        private:
            void verifyParams() {
                bool dimensionless = params_.dimensionless;
                real_T m_prime = params_.mass_dimensionless;
                real_T xg_prime = params_.xg_dimensionless;
                // Check if dimensionless settings are not contradicting
                if (dimensionless) {
                    if (m_prime == 0.0 || xg_prime == 0.0)
                        throw std::invalid_argument("Dimensionless values not set!");
                }
                else {
                    if (!(m_prime == 0.0 && xg_prime == 0.0))
                        throw std::invalid_argument("Dimensionless values are set, but dimensionless is turned off!");
                }
            }

        protected:
            virtual void setupParams() = 0;

        public: //interface
            virtual ~VesselParams() = default;
            virtual void initialize(const AirSimSettings::VehicleSetting* vehicle_setting)
            {
                sensor_storage_.clear();
                sensors_.clear();

                setupParams();
                verifyParams();
                initializeRudders();

                addSensorsFromSettings(vehicle_setting);
            }

            const Params& getParams() const
            {
                return params_;
            }
            Params& getParams()
            {
                return params_;
            }
            SensorCollection& getSensors()
            {
                return sensors_;
            }
            const SensorCollection& getSensors() const
            {
                return sensors_;
            }

            void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
            {
                // use sensors from vehicle settings; if empty list, use default sensors.
                // note that the vehicle settings completely override the default sensor "list";
                // there is no piecemeal add/remove/update per sensor.
                //const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>>& sensor_settings
                //    = vehicle_setting->sensors.size() > 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;
                const auto& sensor_settings = vehicle_setting->sensors;

                getSensorFactory()->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
            }

        protected: //static utility functions for derived classes to use

            virtual const SensorFactory* getSensorFactory() const
            {
                return sensor_factory_.get();
            }

            virtual void fillInertiaMatrix(Matrix3x3r& output_matrix, real_T z_inertia)
            {
                output_matrix << 0, 0, 0,
                    0, 0, 0,
                    0, 0, z_inertia;
            }

            virtual void initializeRudders() = 0;

            virtual void fillMassMatrix(Matrix3x3r& output_matrix, Matrix3x3r& inverse_matrix, real_T mass, real_T z_inertia) = 0;

        private:
            Params params_;
            SensorCollection sensors_; //maintains sensor type indexed collection of sensors
            vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
            std::shared_ptr<const SensorFactory> sensor_factory_;
        };

    }
} //namespace
#endif
