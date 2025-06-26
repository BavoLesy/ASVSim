// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_Hydrodynamics_Wrench_hpp
#define msr_airlib_Hydrodynamics_Wrench_hpp

#include "common/Common.hpp"
#include "physics/Environment.hpp"

namespace msr {
    namespace airlib {

        class HydroDynamicsWrench : public AbstractTau {

        public: //methods
            HydroDynamicsWrench(const Vector3r& position, const Vector3r& normal)
            {
                initialize(position, normal);
            }

            virtual void update(volatile float delta) override
            {
                //update environmental factors before we call base
                updateEnvironmentalFactors();

                //this will in turn call setWrench
                PhysicsBodyVertex::update(delta);

                //perform calculation
                Vector3r forces = computation_method_();

                //update our state
                output_.surge_force = forces(0);
                output_.sway_force = forces(1);
                output_.torque = forces(2);
            }

            virtual void setComputationMethod(std::function<Vector3r()> func) override
            {
                computation_method_ = func;
            }

            virtual void reportState(StateReporter& reporter) override
            {
                reporter.writeValue("Surge-force", output_.surge_force);
                reporter.writeValue("Sway-force", output_.sway_force);
                reporter.writeValue("torque", output_.torque);
            }

        protected:
            virtual void setWrench(Wrench& wrench) override
            {
                wrench.force = Vector3r(output_.surge_force, output_.sway_force, 0.0f);
                wrench.torque = Vector3r(0.0f, 0.0f, output_.torque);
            }

        private:
            void updateEnvironmentalFactors()
            {
                // Pass
            }

        private:
            std::function<Vector3r()> computation_method_;
            Output output_;
        };
    }
} //namespace
#endif
