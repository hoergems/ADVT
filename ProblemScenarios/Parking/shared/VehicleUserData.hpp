#ifndef _UAV_USER_DATA_HPP_
#define _UAV_USER_DATA_HPP_
#include <oppt/opptCore/RobotStateUserData.hpp>

namespace oppt {
class VehicleUserData: public RobotStateUserData {
public:
	using RobotStateUserData::RobotStateUserData;
	_NO_COPY_BUT_MOVE(VehicleUserData)	

	bool insideGoalArea = false;

	bool collides = false;

	int aboveTile = 0;	
};
}

#endif