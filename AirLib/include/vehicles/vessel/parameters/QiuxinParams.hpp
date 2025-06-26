// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_Qiuxin_Parameters_hpp
#define msr_airlib_Qiuxin_Parameters_hpp

#include "vehicles/vessel/VesselParams.hpp"

namespace msr {
    namespace airlib {

        class QiuxinParams : public VesselParams {
        /*
            Based on the Quixin No5 research vessel.
            Paramerers are estimated by You et al.
            https://arxiv.org/abs/2312.05644 
        */

        public:
            QiuxinParams() {
                setupParams();
                initializeRudders();
            }

        protected:
            virtual void setupParams() override {
                auto& params = getParams();
                // Vessel parameters
                params.mass = 187.600;          //kg
                real_T z_inertia = 15.6476;     //kg m^2
                fillInertiaMatrix(params.inertia, z_inertia);
                fillMassMatrix(params.mass_matrix, params.inverse_mass_matrix, params.mass, z_inertia);
                // Hydrodynamical parameters from the paper
                params.X_u = -8.9859;           // kg/s
                params.X_u_u = -31.4285;        // kg/s
                params.X_uuu = -6.8953;         // kg/s
                params.Y_v = -71.9041;          // kg/s
                params.Y_v_v = -77.6429;        // kg/s
                params.Y_vvv = -27.1394;        // kg/s
                params.Y_r_v = -43.2207;        // kg/s
                params.Y_r = -26.0498;          // kg/s
                params.Y_v_r = 26.7652;         // kg/s
                params.Y_r_r = 7.7996;          // kg/s
                params.N_v = -14.8953;          // kg/s
                params.N_v_v = -1.6306;         // kg/s
                params.N_r_v = 8.7911;          // kg/s
                params.N_r = -26.7122;          // kg/s
                params.N_r_r = -9.8284;         // kg/s
                params.N_rrr = -9.2320;         // kg/s
                params.N_v_r = -2.3474;         // kg/s
                params.length = 2.152;          // m
            }

            virtual void fillMassMatrix(Matrix3x3r& output_matrix, Matrix3x3r& inverse_matrix, real_T mass, real_T z_inertia) {
                output_matrix << 138.0574, 0, 0,
                    0, 106.6003, 1.1254,
                    0, -16.0598, z_inertia;
                inverse_matrix = output_matrix.inverse();
            }

            virtual void initializeRudders() {
                auto& params = getParams();
                params.rudder_params.emplace_back(2.0, 0.5, 100, 0.0, -0.8, 0.163);
                params.rudder_params.emplace_back(2.0, 0.5, 100, 0.0, -0.8, -0.163);
            }
        };

    }
} // Namespace

#endif