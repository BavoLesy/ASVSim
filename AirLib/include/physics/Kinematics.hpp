// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_Kinematics_hpp
#define airsim_core_Kinematics_hpp

#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "common/CommonStructs.hpp"

namespace msr { namespace airlib {

class Kinematics : public UpdatableObject {
public:
    struct State {
        Pose pose;
        Twist twist;
        Wrench wrench;
        Accelerations accelerations;

        static State zero()
        {
            State zero_state;
            zero_state.pose.position = Vector3r::Zero();
            zero_state.pose.orientation = Quaternionr::Identity();
            zero_state.twist = Twist::zero();
            zero_state.wrench = Wrench::zero();
            zero_state.accelerations = Accelerations::zero();

            return zero_state;
        }
    };

    Kinematics(const State& initial = State::zero())
    {
        initialize(initial);
    }
    void initialize(const State& initial)
    {
        initial_ = initial;
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
        UpdatableObject::reset();

        current_ = initial_;
    }

    virtual void update(float delta = 0) override
    {
        UpdatableObject::update(delta);

        //nothing to do because next state should be updated 
        //by physics engine. The reason is that final state
        //needs to take in to account state of other objects as well,
        //for example, if collision occurs
    }
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("Position", current_.pose.position);
        reporter.writeValue("Orientation", current_.pose.orientation);
        reporter.writeValue("Lin-Vel", current_.twist.linear);
        reporter.writeValue("Lin-Accl", current_.accelerations.linear);
        reporter.writeValue("Ang-Vel", current_.twist.angular);
        reporter.writeValue("Ang-Accl", current_.accelerations.angular);
        //added for vessel dynamics calculation 
        reporter.writeValue("Wrench-Force", current_.wrench.force);
        reporter.writeValue("Wrench-Torque", current_.wrench.torque);

    }
    //*** End: UpdatableState implementation ***//

    const Pose& getPose() const
    {
        return current_.pose;
    }
    void setPose(const Pose& pose)
    {
        current_.pose = pose;
    }
    const Twist& getTwist() const
    {
        return current_.twist;
    }
    void setTwist(const Twist& twist)
    {
        current_.twist = twist;
    }
    const Wrench& getWrench() const
    {
        return current_.wrench;
    }
    void setWrench(const Wrench& wrench)
    {
        current_.wrench = wrench;
    }
    const State& getState() const
    {
        return current_;
    }
    void setState(const State& state)
    {
        current_ = state;
    }
    const State& getInitialState() const
    {
        return initial_;
    }

private: //fields
    State initial_;
    State current_;
};

}} //namespace
#endif
