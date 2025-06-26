// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_Mariner_Parameters_hpp
#define msr_airlib_Mariner_Parameters_hpp

#include "vehicles/vessel/VesselParams.hpp"

namespace msr {
    namespace airlib {

        class MarinerParams : public VesselParams {
        public:
            MarinerParams() {
                setupParams();
                initializeRudders();
            }

        protected:
            virtual void setupParams() override {
                auto& params = getParams();

                // Main dimensions
                params.length = 160.93; // [m]
                params.design_speed = 7.7175; // [m/s] (15 knots)
                params.mass = 798e-5; // [kg] (non-dimensionalized)
                params.xg_dimensionless = -0.023; // [L]
                real_T z_inertia = 39.2e-5; // [kg*m^2] (non-dimensionalized)
                fillInertiaMatrix(params.inertia, z_inertia);
                fillMassMatrix(params.mass_matrix, params.inverse_mass_matrix, params.mass, z_inertia);

                // Hydrodynamic derivatives (non-dimensionalized)
                params.X_u = -184e-5;
                params.X_u_u = -110e-5;
                params.X_uuu = -215e-5;
                params.X_v_v = -899e-5;
                params.X_r_r = 18e-5;
                params.X_r_v = 798e-5;
                params.X_d_d = -95e-5;
                params.X_v_d = 93e-5;
                params.X_r_d = 0.0; // Not used in original model
                // Additional X terms
                params.X_uuu = -215e-5;
                params.X_udd = -190e-5;
                params.X_uvd = 93e-5;

                // Y-coefficients
                params.Y_v = -1160e-5;
                params.Y_r = -499e-5;
                params.Y_vvv = -8078e-5;
                params.Y_v_v_r = 15356e-5;
                params.Y_v_r = 0.0; // Not used in original model
                params.Y_r_r = 0.0; // Not used in original model
                params.Y_rrr = 0.0; // Not used in original model
                params.Y_d = 278e-5;
                params.Y_d_d = 0.0; // Not used in original model
                params.Y_d_d_d = -90e-5;
                params.Y_u_d = 556e-5;
                params.Y_v_d_d = -4e-5;
                params.Y_v_v_d = 1190e-5;
                params.Y_r_d_d = 0.0;
                params.Y_r_r_d = 0.0;
                // Additional Y terms
                params.Y_vu = -1160e-5;
                params.Y_ru = -499e-5;
                params.Y_ud = 556e-5;
                params.Y_u_u_d = 278e-5;
                params.Y_0 = -4e-5;
                params.Y_0_u = -8e-5;
                params.Y_0_u_u = -4e-5;

                // N-coefficients
                params.N_v = -264e-5;
                params.N_r = -166e-5;
                params.N_vvv = 1636e-5;
                params.N_v_v_r = -5483e-5;
                params.N_v_r = 0.0;
                params.N_r_r = 0.0;
                params.N_rrr = 0.0;
                params.N_d = -139e-5;
                params.N_d_d = 0.0;
                params.N_d_d_d = 45e-5;
                params.N_u_d = -278e-5;
                params.N_v_d_d = 13e-5;
                params.N_v_v_d = -489e-5;
                params.N_r_d_d = 0.0;
                params.N_r_r_d = 0.0;
                // Additional N terms
                params.N_vu = -264e-5;
                params.N_ru = -166e-5;
                params.N_ud = -278e-5;
                params.N_u_u_d = -139e-5;
                params.N_0 = 3e-5;
                params.N_0_u = 6e-5;
                params.N_0_u_u = 3e-5;

                // Dimensionless
                params.dimensionless = true;
				params.large_vessel = true;
                params.mass_dimensionless = params.mass;
                // xg_dimensionless already set above
            }

            virtual void fillMassMatrix(Matrix3x3r& output_matrix, Matrix3x3r& inverse_matrix, real_T mass, real_T z_inertia) override {
                // Masses and moments of inertia (from MATLAB script)
                real_T xG = -0.023;
                real_T Xudot = -42e-5;
                real_T Yvdot = -748e-5;
                real_T Yrdot = -9.354e-5;
                real_T Nvdot = 4.646e-5;
                real_T Nrdot = -43.8e-5;

                real_T m11 = mass - Xudot;
                real_T m22 = mass - Yvdot;
                real_T m23 = mass * xG - Yrdot;
                real_T m32 = mass * xG - Nvdot;
                real_T m33 = z_inertia - Nrdot;

                output_matrix << m11, 0, 0,
                    0, m22, m23,
                    0, m32, m33;
                inverse_matrix = output_matrix.inverse();
            }

            virtual void initializeRudders() override {
                auto& params = getParams();
                // Mariner: single rudder, typical values
                params.rudder_params.emplace_back(2.0, 10.0, 2000e3, 0.0, -100.0, 0.0);
            }
        };

    } // namespace airlib
} // namespace msr

#endif
