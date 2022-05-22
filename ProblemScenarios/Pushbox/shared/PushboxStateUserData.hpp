#ifndef _PUSHBOX_STATE_USER_DATA_HPP_
#define _PUSHBOX_STATE_USER_DATA_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {
class PushboxStateUserData: public OpptUserData {
public:
	using OpptUserData::OpptUserData;

	RobotStateSharedPtr previousState = nullptr;

	bool isGoalState = false;
	bool isInCollision = false;
	bool pushed = false;
};
}

#endif