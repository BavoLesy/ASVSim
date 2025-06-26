// Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
// Licensed under the MIT License.
// Author: Siemen Herremans

#ifndef msr_airlib_abstract_hydrodynamics_hpp
#define msr_airlib_abstract_hydrodynamics_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"
#include "VesselParams.hpp"
#include "common/SteppableClock.hpp"
#include "common/UpdatableObject.hpp"

namespace msr { namespace airlib {
	class AbstractHydrodynamics {
		/*
		This class has two main tasks:
			- Compute the coriolis matrix
			- Compute the damping matrix
		*/
	public:
		virtual void computeCoriolis() = 0;
		virtual void computeDamping() = 0;
		
		Vector3r getDampingForce() const {
			return dampingForce_;
		}

		Vector3r getCoriolisForce() const {
			return coriolisForce_;
		}
		void setVesselParameters(VesselParams* params) {
			this->parameters_ = params;
		}

		const VesselParams* getVesselParameters() const {
			return this->parameters_;
		}

		const Vector3r& getNuDot() const {
			return nu_dot_;
		}

		const Vector3r& getEtaDot() const {
			return eta_dot_;
		}

		void setRudderAngle(float angle) {
			rudder_angle_ = angle;
		}

		void updateState(const Kinematics::State& state, const Vector2r& current) {
			nu_ = Vector3r(state.twist.linear.x(), state.twist.linear.y(), state.twist.angular.z());
			heading_ = toYaw(state.pose.orientation);
			// Ocean currents must be expressed in local frame as well
			current_ = VectorMath::transformToBodyFrame(Vector3r(current.x(), current.y(), 0.0), state.pose.orientation, false);
		}

		// Returns yaw in radians
		static float toYaw(const Quaternionr& q) {
			double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
			double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
			return std::atan2(siny_cosp, cosy_cosp);
		}

		virtual ~AbstractHydrodynamics() = default;

	protected:
		VesselParams* parameters_;
		Vector3r dampingForce_, coriolisForce_;
		Vector3r nu_, current_, nu_dot_, eta_dot_;
		float heading_ = 0.0f;		// In radians
		float rudder_angle_ = 0.0f; // In radians

	};

}} // namespace


#endif