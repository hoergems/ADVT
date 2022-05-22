#ifndef _UAV_STATE_HPP_
#define _UAV_STATE_HPP_
#include <oppt/opptCore/core.hpp>
#include <oppt/robotHeaders/RobotState.hpp>

namespace oppt {
class VehicleState: public VectorState {
public:
	using VectorState::VectorState;

	FloatType x() const {
		return state_[0];
	}

	FloatType &x() {
		return state_[0];
	}

	FloatType y() const {
		return state_[1];
	}

	FloatType &y() {
		return state_[1];
	}

	FloatType z() const {
		return state_[2];
	}

	FloatType &z() {
		return state_[2];
	}

	FloatType yaw() const {
		return state_[3];
	}

	FloatType &yaw() {
		return state_[3];
	}

	FloatType velocity() const {
		return state_[4];
	}

	FloatType &velocity() {
		return state_[4];
	}

};
}

#endif