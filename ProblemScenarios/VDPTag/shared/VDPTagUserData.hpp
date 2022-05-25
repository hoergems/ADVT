#ifndef _VDP_TAG_USER_DATA_HPP_
#define _VDP_TAG_USER_DATA_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {
class VDPTagUserData: public RobotStateUserData {
public:
	using RobotStateUserData::RobotStateUserData;

	FloatType dist = 0.0;

	int activeBeam = 0;

};
}

#endif