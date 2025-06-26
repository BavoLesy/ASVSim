// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_Tanker_Parameters_hpp
#define msr_airlib_Tanker_Parameters_hpp

#include "vehicles/vessel/VesselParams.hpp"

namespace msr {
    namespace airlib {

        class TankerParams : public VesselParams {
            /*
                Based on the Quixin No5 research vessel.
                Paramerers are estimated by You et al.
                https://arxiv.org/abs/2312.05644
            */

        public:
            TankerParams() {
                setupParams();
                initializeRudders();
            }

        protected:
            virtual void setupParams() override {
                auto& params = getParams();
                params.dimensionless = true;
                params.design_speed = 7.956;        // [m/s], a little over 15 knots
                // Vessel parameters
                real_T actual_mass = 312622000.0;   // [kg]
                params.mass = 0.01913835;           // [-], mass / (0.5 * rho * L^3)
                real_T real_z_inertia = 2e12;       // [kg/m^2]
                real_T z_inertia = 0.00119567993;   // [-], z_inertia / (0.5 * rho * L ^ 5)
                fillInertiaMatrix(params.inertia, z_inertia);
                fillMassMatrix(params.mass_matrix, params.inverse_mass_matrix, params.mass, z_inertia);
                params.length = 320.00; // m

                // Hydrodynamical parameters from the paper (not including rudder here)
                
                // X-coefficients
                params.X_u = -0.0022;
                params.X_u_u = 0.0015;
                params.X_uuu = 0.0; // Assuming a default value as it's not in the table
                params.X_v_v = 0.00159;
                params.X_r_r = 0.000338;
                params.X_v_r = 0.01391;
                params.X_d_d = 0.00272;
                params.X_v_d = 0.001609;
                params.X_r_d = 0.001034;

                // Y-coefficients
                params.Y_v = -0.01902;
                params.Y_v_v = 0.000639;
                params.Y_vvv = -0.1287;
                params.Y_r = 0.005719;
                params.Y_v_r = 0.0; // Assuming a default value as it's not in the table
                params.Y_r_r = 0.000002;
                params.Y_rrr = -0.000048;
                params.Y_d = 0.00408;
                params.Y_d_d = 0.000114;
                params.Y_d_d_d = -0.003059;
                params.Y_v_r_r = -0.02429;
                params.Y_v_v_r = 0.0211;
                params.Y_u_d = -0.00456;
                params.Y_v_d_d = 0.00326;
                params.Y_v_v_d = 0.003018;
                params.Y_r_d_d = 0.002597;
                params.Y_r_r_d = 0.000895;

                // N-coefficients
                params.N_v = -0.007886;
                params.N_v_v = -0.000308;
                params.N_vvv = 0.00175;
                params.N_r = -0.003701;
                params.N_r_r = -0.000002;
                params.N_rrr = -0.000707;
                params.N_d = -0.001834;
                params.N_d_d = -0.000056;
                params.N_d_d_d = 0.001426;
                params.N_v_r_r = 0.003726;
                params.N_v_v_r = -0.019;
                params.N_u_d = 0.00232;
                params.N_v_d_d = -0.001504;
                params.N_v_v_d = 0.001406;
                params.N_r_d_d = 0.001191;
                params.N_r_r_d = 0.000398;
            }

            virtual void fillMassMatrix(Matrix3x3r& output_matrix, Matrix3x3r& inverse_matrix, real_T mass, real_T z_inertia) {
                real_T xg = 0.0348; // [-]
                real_T X_u_dot = -0.001135;
                real_T Y_v_dot = -0.014508;
                real_T Y_r_dot = -0.001209;
                real_T N_v_dot = -0.000588;
                real_T N_r_dot = -0.000564;
                output_matrix << mass - X_u_dot, 0, 0,
                    0, mass - Y_v_dot, mass * xg - Y_r_dot,
                    0, mass * xg - N_v_dot, z_inertia - N_r_dot;
                inverse_matrix = output_matrix.inverse();
            }

            virtual void initializeRudders() {
                auto& params = getParams();
                params.rudder_params.emplace_back(2.0, 0.5, 500e3, 0.0, -100.0, 0.0);
            }
        };

    }
} // Namespace

#endif