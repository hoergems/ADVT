#ifndef _VDP_TAG_USER_DATA_HPP_
#define _VDP_TAG_USER_DATA_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {
class VDPTagUserData: public RobotStateUserData {
public:
	using RobotStateUserData::RobotStateUserData;

	FloatType dist = 0.0;

	FloatType numScans = 0.0;

	FloatType timeSinceLastScan = 0.0;
};
}

#endif