#ifndef _SENSOR_PLACEMENT_USER_DATA_HPP_
#define _SENSOR_PLACEMENT_USER_DATA_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {
class SensorPlacementUserData: public RobotStateUserData {
public:
	using RobotStateUserData::RobotStateUserData;

	bool reachedGoal = false;

	bool collides = false;

	bool eeTouchesWall = false;

	geometric::Pose eePose;

};

}

#endif