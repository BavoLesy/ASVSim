// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_fossen_current_hpp
#define msr_airlib_fossen_current_hpp


#include "vehicles/vessel/AbstractHydrodynamics.hpp"

namespace msr {
    namespace airlib {
        class FossenCurrent : public AbstractHydrodynamics {

            virtual void computeCoriolis() override
            {
                const VesselParams::Params& params = parameters_->getParams();
                Vector3r relative_velocities = nu_ - current_;

                real_T u = relative_velocities(0);
                real_T v = relative_velocities(1);
                real_T r = relative_velocities(2);

                real_T m_11 = params.mass_matrix(0, 0);
                real_T m_22 = params.mass_matrix(1, 1);
                real_T m_23 = params.mass_matrix(1, 2);
                real_T m_32 = params.mass_matrix(2, 1);
                real_T c13 = -m_22 * v - 0.5 * (m_23 + m_32) * r;
                real_T c23 = m_11 * u;
                real_T c31 = -c13;
                real_T c32 = -c23;

                Matrix3x3r coriolisMatrix = Matrix3x3r::Zero();
                coriolisMatrix << 0, 0, c13,
                    0, 0, c23,
                    c31, c32, 0;

                coriolisForce_ = coriolisMatrix * relative_velocities;
            }

            virtual void computeDamping() override
            {
                const VesselParams::Params& params = parameters_->getParams();
                Vector3r relative_velocities = nu_ - current_;
                
                real_T u = relative_velocities(0);
                real_T v = relative_velocities(1);
                real_T r = relative_velocities(2);

                real_T d_11 = -params.X_u - params.X_u_u * abs(u) - params.X_uuu * u * u;
                real_T d_22 = -params.Y_v - params.Y_v_v * abs(v) - params.Y_r_v * abs(r) - params.Y_vvv * v * v;
                real_T d_23 = -params.Y_r - params.Y_v_r * abs(v) - params.Y_r_r * abs(r);
                real_T d_32 = -params.N_v - params.N_v_v * abs(v) - params.N_r_v * abs(r);
                real_T d_33 = -params.N_r - params.N_v_r * abs(v) - params.N_r_r * abs(r) - params.N_rrr * r * r;

                Matrix3x3r dampingMatrix = Matrix3x3r::Zero();
                dampingMatrix << d_11, 0, 0,
                    0, d_22, d_23,
                    0, d_32, d_33;

                dampingForce_ = dampingMatrix * relative_velocities;
            }

        };

    }
} // namespace


#endif