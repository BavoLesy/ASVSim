// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_Cybership2_Parameters_hpp
#define msr_airlib_Cybership2_Parameters_hpp

#include "vehicles/vessel/VesselParams.hpp"

namespace msr {
    namespace airlib {

        class Cybership2Params : public VesselParams {

        public:
            Cybership2Params() {
                setupParams();
                initializeRudders();
            }

        protected:
            virtual void setupParams() override {
                auto& params = getParams();
                // Vessel parameters
                params.mass = 23.800;          //kg
                real_T z_inertia = 1.760;     //kg m^2
                fillInertiaMatrix(params.inertia, z_inertia);
                fillMassMatrix(params.mass_matrix, params.inverse_mass_matrix, params.mass, z_inertia);
                // Hydrodynamical parameters from the paper
                params.X_u = -2.0;          // kg/s
                params.X_u_u = -1.32742;        // kg/s
                params.X_uuu = -5.86643;        // kg/s
                params.Y_v = -7.0;          // kg/s
                params.Y_v_v = -36.47287;       // kg/s
                params.Y_r_v = -0.805;          // kg/s
                params.Y_r = -0.1;            // kg/s
                params.Y_v_r = -0.845;          // kg/s
                params.Y_r_r = -3.450;          // kg/s
                params.N_r_v = 0.130;           // kg/s
                params.N_r = -0.5;            // kg/s
                params.N_r_r = -0.750;          // kg/s
                params.N_v_r = 0.080;           // kg/s
                params.length = 1.255;          // m
                params.N_v_v = 3.95645;
                params.N_v = 0.03130;
                // Non-declared params
                params.Y_vvv = 0.0;
                params.N_rrr = -0.0;
            }

            virtual void fillMassMatrix(Matrix3x3r& output_matrix, Matrix3x3r& inverse_matrix, real_T mass, real_T z_inertia) {
                real_T xg = 0.046;
                real_T X_u_dot = -2.0;
                real_T Y_v_dot = -10.0;
                real_T Y_r_dot = -0.0;
                real_T N_v_dot = -0.0;
                real_T N_r_dot = -1.0;
                output_matrix << mass - X_u_dot, 0, 0,
                    0, mass - Y_v_dot, mass*xg - Y_r_dot,
                    0, mass*xg - N_v_dot, z_inertia - N_r_dot;
                inverse_matrix = output_matrix.inverse();
            }

            virtual void initializeRudders() {
                auto& params = getParams();
                params.rudder_params.emplace_back(2.0, 0.5, 3.0, M_PI / 6, -0.3, 0.0);
            }
        };

    }
} // Namespace

#endif