// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef airsimcore_thruster_hpp
#define airsimcore_thruster_hpp

#include <limits>
#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include "common/FirstOrderFilter.hpp"
#include "AbstractTau.hpp"
#include "ThrusterParams.hpp"

namespace msr { namespace airlib {

// Rudder gets control signal as input (Desired normalized thrust (N) and normalized desired rudder angle (radians)) 
// which causes change in rotation speed and turning direction and ultimately produces force (surge) and torque (yaw) 
// as output. Currently, we consider an under_actuated vessel with 2 controllable degrees.
class Thruster : public AbstractTau {

    using AbstractTau::initialize;      // No overloading across scopes in c++

public: //methods

	Thruster()
	{
		//allow default constructor with later call for initialize
	}

    Thruster(const Vector3r& position, const Vector3r& normal, const ThrusterParams& params, const Environment* environment, uint id = -1)
    {
        initialize(position, normal, params, environment, id);
    }

    void initialize(const Vector3r& position, const Vector3r& normal, 
        const ThrusterParams& params, const Environment* environment, uint id = -1)
    {
        id_ = id;
        params_ = params;
        environment_ = environment;
        
        thrust_signal_filter_.initialize(params_.thrust_signal_filter_tc, 0, 0);
        angle_signal_filter_.initialize(params_.angle_signal_filter_tc, 0.5, 0.5);

        AbstractTau::initialize(position, normal);   //call base initializer
    }
    
    //0 to 1 - will be scaled to 0 to max_speed
    void setControlSignal(real_T force_signal, real_T angle_signal) override
    {
        thrust_signal_filter_.setInput(Utils::clip(force_signal, 0.0f, 1.0f));
        angle_signal_filter_.setInput(Utils::clip(angle_signal, 0.0f, 1.0f));
    }

       
    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
        PhysicsBodyVertex::resetImplementation();

        //update environmental factors before we call base
        updateEnvironmentalFactors();

        thrust_signal_filter_.reset();
        angle_signal_filter_.reset();

        setOutput(output_, params_, thrust_signal_filter_, angle_signal_filter_);
    }

    virtual void update(float delta = 0) override
    {
        //update environmental factors before we call base
        updateEnvironmentalFactors();

        //this will in turn call setWrench
        PhysicsBodyVertex::update(delta);

        //update our state
        setOutput(output_, params_, thrust_signal_filter_, angle_signal_filter_);

        //update filter - this should be after so that first output is same as initial
        thrust_signal_filter_.update(delta);
        angle_signal_filter_.update(delta);
    }

    virtual void reportState(StateReporter& reporter) override
    {
        reporter.writeValue("Thrust-in", output_.thrust_signal_input);
        reporter.writeValue("Thrust-fl", output_.thrust_signal_filtered);
        reporter.writeValue("Angle-in", output_.angle_signal_input);
        reporter.writeValue("Angle-fl", output_.angle_signal_filtered);
        reporter.writeValue("Surge-force", output_.surge_force);
		reporter.writeValue("Sway-force", output_.sway_force);
        reporter.writeValue("torque", output_.torque);
    }
    //*** End: UpdatableState implementation ***//


protected:
    virtual void setWrench(Wrench& wrench) override
    {
        // This represents the rudder at an angle
        wrench.force =  Vector3r(output_.surge_force, output_.sway_force, 0.0f);
        wrench.torque = Vector3r(0.0f, 0.0f, output_.torque);
    }

private: //methods
    static void setOutput(Output& output, const ThrusterParams& params, const FirstOrderFilter<real_T>& thrust_signal_filter, const FirstOrderFilter<real_T>& angle_signal_filter)
    {
        output.thrust_signal_input = thrust_signal_filter.getInput();
        output.thrust_signal_filtered = thrust_signal_filter.getOutput();
        output.angle_signal_input = angle_signal_filter.getInput();
        output.angle_signal_filtered = angle_signal_filter.getOutput();
        // Split up with simple basis vector decomposition
        basisDecomposition(output, params);
    }

    void updateEnvironmentalFactors()
    {
        // Pass
    }

    inline static void basisDecomposition(Output& output, const ThrusterParams& params) {
        float force = output.thrust_signal_filtered * params.max_thrust;
        float angle = (output.angle_signal_filtered - 0.5) * 2 * params.max_rudder_angle;
        output.surge_force = force * std::cos(angle);
        output.sway_force  = force * std::sin(angle);
        output.torque = params.l_x* output.sway_force + params.l_y * output.surge_force;
    }

private: //fields
    uint id_; //only used for debug messages
    ThrusterParams params_;
    FirstOrderFilter<real_T> thrust_signal_filter_;
    FirstOrderFilter<real_T> angle_signal_filter_;
    const Environment* environment_ = nullptr;
};


}} //namespace
#endif
