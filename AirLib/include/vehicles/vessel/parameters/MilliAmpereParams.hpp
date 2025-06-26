// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_Milli_Ampere_Parameters_hpp
#define msr_airlib_Milli_Ampere_Parameters_hpp

#include "vehicles/vessel/VesselParams.hpp"

namespace msr {
    namespace airlib {

        class MilliAmpereParams : public VesselParams {
        /*
            Based on the MilliAmpere research vessel.
            Paramerers are estimated by Pedersen et al.
            https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2625699
        */

        public:
            MilliAmpereParams() {
                setupParams();
                initializeRudders();
            }

        protected:
            virtual void setupParams() override {
                auto& params = getParams();
                // Vessel parameters
                params.mass = 2400;             //kg
                real_T z_inertia = 5068.910;    //kg m^2
                fillInertiaMatrix(params.inertia, z_inertia);
                fillMassMatrix(params.mass_matrix, params.inverse_mass_matrix, params.mass, z_inertia);
                // Hydrodynamical parameters
                params.X_u = -27.632;           //kg s^-1
                params.X_u_u = -110.064;        //kg m^-1
                params.X_uuu = -13.965;         //kg s m^-1
                params.Y_v = -52.947;           //kg s^-1
                params.Y_v_v = -116.486;        //kg m^-1
                params.Y_vvv = -24.313;         //kg s m^-1
                params.Y_r_v = -1540.383;       //kg
                params.Y_r = 24.732;            //kg m s^-1 
                params.Y_v_r = 572.141;         //kg
                params.Y_r_r = -115.456;        //kg m
                params.N_v = 3.5241;            //kg m s^-1
                params.N_v_v = -0.832;          //kg
                params.N_r_v = 336.827;         //kg m
                params.N_r = -122.860;          //kg m^2 s^-1
                params.N_r_r = -874.428;        //kg m^2
                params.N_rrr = 0.000;           //kg m^2 s
                params.N_v_r = -121.957;        //kg m
                params.length = 5.0;            //m
            }

            virtual void fillMassMatrix(Matrix3x3r& output_matrix, Matrix3x3r& inverse_matrix, real_T mass, real_T z_inertia) {
                output_matrix << mass, 0, 0,
                    0, mass, 50.0,
                    0, 50.0, z_inertia;
                inverse_matrix = output_matrix.inverse();
            }

            virtual void initializeRudders() {
                auto& params = getParams();
                params.rudder_params.emplace_back(2.0, 0.5, 500, M_PI, -1.8, 0.0);
                params.rudder_params.emplace_back(2.0, 0.5, 500, M_PI, 1.8, 0.0);
            }
        };

    }
} // Namespace

#endif