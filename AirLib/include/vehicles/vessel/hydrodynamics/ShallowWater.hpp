// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

// Source: D. Clarke (1997), "The Shallow Water Effect on Linear Derivatives"

#ifndef msr_airlib_ShallowWater_hpp
#define msr_airlib_ShallowWater_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/vessel/VesselParams.hpp"
#include <algorithm>

namespace msr {
    namespace airlib {

        struct ShallowWaterCorrections {
            // See Eqn. (5) of Clarke (1997)
            real_T Yv_dot = 1.0;
            real_T Nr_dot = 1.0;
            real_T Yv = 1.0;    
            real_T Yr = 1.0;   
            real_T Nv = 1.0;   
            real_T Nr = 1.0;

        };

        class ClarkeShallowWater {

        public:
            
            ClarkeShallowWater(VesselParams* params) {
                this->parameters_ = params;
            }

            const ShallowWaterCorrections computeCorrection(real_T depth) {
                if (std::abs(depth) < std::numeric_limits<real_T>::epsilon())
                    return ShallowWaterCorrections();
                
                ShallowWaterCorrections corrections;
                real_T beam = parameters_->getParams().beam;
                real_T draft = parameters_->getParams().draft;

                real_T hT = std::max(depth / draft, (real_T)1.25);
                real_T F = hT - 1.0;
                real_T BT = beam / draft;


                real_T F2 = F * F;
                real_T F3 = F * F * F;
                real_T K0 = 1.0 + (0.0774 / F2) - (0.0151 / F3);
                real_T K1 = (-0.0125 / F) + (0.1674 / F2) - (0.0199 / F3);
                real_T K2 = 0.0431 / F;


                // 3. Apply the geometric weightings for each specific derivative
                corrections.Yv_dot = K0 + (2.0 / 3.0) * K1 * BT + (8.0 / 15.0) * K2 * (BT * BT);
                corrections.Nr_dot = K0 + (2.0 / 5.0) * K1 * BT + (24.0 / 105.0) * K2 * (BT * BT);

                corrections.Yv = K0 + K1 * BT + K2 * (BT * BT);
                corrections.Nv = K0 + (2.0 / 3.0) * K1 * BT + (8.0 / 15.0) * K2 * (BT * BT);
                corrections.Yr = K0 + (2.0 / 3.0) * K1 * BT + (8.0 / 15.0) * K2 * (BT * BT);
                corrections.Nr = K0 + (1.0 / 2.0) * K1 * BT + (1.0 / 3.0) * K2 * (BT * BT);

                return corrections;
            }

        protected:
            VesselParams* parameters_;
        };

    }
}

#endif