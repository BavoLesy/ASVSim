// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_Mariner_Abkowitz_hpp
#define msr_airlib_Mariner_Abkowitz_hpp

#include "vehicles/vessel/AbstractHydrodynamics.hpp"

namespace msr {
    namespace airlib {

        class MarinerHydrodynamics : public AbstractHydrodynamics {

        public:
            virtual void computeCoriolis() override {
                coriolisForce_ = Vector3r::Zero();
            }

			virtual void computeDamping() override {
                const VesselParams::Params& params = parameters_->getParams();

                real_T u = nu_(0);
                real_T v = nu_(1);
                real_T r = nu_(2);

                real_T delta = rudder_angle_;

                // Dimensions  
                const real_T L = params.length;
                const real_T U0 = params.design_speed.value_or(7.7175);

                real_T U = std::sqrt((U0 + u) * (U0 + u) + v * v);
                if (U < 1e-6) U = 1e-6; // avoid division by zero  

                // Non-dimensionalized states  
                real_T u_nd = u / U;
                real_T v_nd = v / U;
                real_T r_nd = r * L / U;

                // Rudder saturation 
                const real_T delta_max = 40.0 * M_PI / 180.0;
                if (std::abs(delta) > delta_max)
                    delta = std::copysign(delta_max, delta);

                // Surge
                real_T X = params.X_u * u_nd
                    + params.X_u_u * u_nd * u_nd
                    + params.X_uuu * u_nd * u_nd * u_nd
                    + params.X_v_v * v_nd * v_nd
                    + params.X_r_r * r_nd * r_nd
                    + params.X_r_v * r_nd * v_nd
                    + params.X_d_d * delta * delta
                    + params.X_udd * u_nd * delta * delta
                    + params.X_v_d * v_nd * delta
                    + params.X_uvd * u_nd * v_nd * delta;

                // Sway  
                real_T Y = params.Y_v * v_nd
                    + params.Y_r * r_nd
                    + params.Y_vvv * v_nd * v_nd * v_nd
                    + params.Y_v_v_r * v_nd * v_nd * r_nd
                    + params.Y_vu * v_nd * u_nd
                    + params.Y_ru * r_nd * u_nd
                    + params.Y_d * delta
                    + params.Y_d_d_d * delta * delta * delta
                    + params.Y_u_d * u_nd * delta
                    + params.Y_u_u_d * u_nd * u_nd * delta
                    + params.Y_v_d_d * v_nd * delta * delta
                    + params.Y_v_v_d * v_nd * v_nd * delta
                    + (params.Y_0 + params.Y_0_u * u_nd + params.Y_0_u_u * u_nd * u_nd);

                // Yaw
                real_T N = params.N_v * v_nd
                    + params.N_r * r_nd
                    + params.N_vvv * v_nd * v_nd * v_nd
                    + params.N_v_v_r * v_nd * v_nd * r_nd
                    + params.N_vu * v_nd * u_nd
                    + params.N_ru * r_nd * u_nd
                    + params.N_d * delta
                    + params.N_d_d_d * delta * delta * delta
                    + params.N_u_d * u_nd * delta
                    + params.N_u_u_d * u_nd * u_nd * delta
                    + params.N_v_d_d * v_nd * delta * delta
                    + params.N_v_v_d * v_nd * v_nd * delta
                    + (params.N_0 + params.N_0_u * u_nd + params.N_0_u_u * u_nd * u_nd);

                const Matrix3x3r& mass_matrix = params.mass_matrix;
                real_T m11 = mass_matrix(0, 0);
                real_T m22 = mass_matrix(1, 1);
                real_T m23 = mass_matrix(1, 2);
                real_T m32 = mass_matrix(2, 1);
                real_T m33 = mass_matrix(2, 2);

                real_T detM22 = m22 * m33 - m23 * m32;

                eta_dot_ << (std::cos(heading_) * (U0 / U + u_nd) - std::sin(heading_) * v_nd) * U,
                    (std::sin(heading_) * (U0 / U + u_nd) + std::cos(heading_) * v_nd)* U,
                    r_nd* (U / L);

                nu_dot_ << X * ((U * U) / L) / m11,
                     -(-m33 * Y + m23 * N) * (U * U / L) / detM22,
                     (-m32 * Y + m22 * N)* (U * U / (L * L)) / detM22;
            }
        };

    } // namespace airlib
} // namespace msr

#endif