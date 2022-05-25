#ifndef _VDPTag_STATE_HPP_
#define _VDPTag_STATE_HPP_
#include <oppt/opptCore/core.hpp>
#include <oppt/robotHeaders/RobotState.hpp>
#include "VDPTagUserData.hpp"

namespace oppt {
class VDPTagState: public VectorState {
public:
	VDPTagState(const VectorFloat &stateVec):
		VectorState(stateVec),
		agentPos_( {stateVec[0], stateVec[1]}),
	targetPos_({stateVec[2], stateVec[3]}) {

	}

	VDPTagState(const Vector2f &agentPos, const Vector2f &targetPos):
		VectorState( {agentPos[0], agentPos[1], targetPos[0], targetPos[1]}),
	agentPos_(agentPos),
	targetPos_(targetPos) {

	}

	RobotStateSharedPtr makeCopy() const {
		RobotStateSharedPtr copied(new VDPTagState(state_));
		auto ud = getUserData()->as<VDPTagUserData>();

		OpptUserDataSharedPtr udNew(new VDPTagUserData);
		udNew->as<VDPTagUserData>()->dist = ud->as<VDPTagUserData>()->dist;
		udNew->as<VDPTagUserData>()->activeBeam = ud->as<VDPTagUserData>()->activeBeam;
		copied->setUserData(udNew);
		return copied;
	}

	Vector2f agentPos() const {
		return agentPos_;
	}

	Vector2f &agentPos() {
		return agentPos_;
	}

	Vector2f targetPos() const {
		return targetPos_;
	}

	Vector2f &targetPos() {
		return targetPos_;
	}

	virtual void print(std::ostream& os) const override {
		os << "Agent: (" << agentPos_[0] << ", " << agentPos_[1] << "), Target: (" << targetPos_[0] << ", " << targetPos_[1] << ")";
	}

private:
	Vector2f agentPos_;
	Vector2f targetPos_;
};
}

#endif