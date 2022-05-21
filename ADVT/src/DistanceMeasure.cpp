#include "DistanceMeasure.hpp"

namespace oppt {
FloatType EuclideanDistanceMeasure::operator()(const VectorFloat &a1, const VectorFloat &a2) const {
	return math::euclideanDistance(a1, a2);
}

FloatType EuclideanDistanceMeasure::operator()(const Vectordf &a1, const Vectordf &a2) const {
	return (a2 - a1).norm();
}

FloatType VDPMeasure::operator()(const VectorFloat &a1, const VectorFloat &a2) const {
	if ((a1[1] > 0.5 and a2[1] > 0.5) or (a1[1] < 0.5 or a2[1] < 0.5)) {
		FloatType angleDiff = std::fabs(a1[0] - a2[0]);
		return std::min(angleDiff, 2.0*M_PI - angleDiff);
	}

	return std::numeric_limits<FloatType>::infinity();
}

FloatType VDPMeasure::operator()(const Vectordf &a1, const Vectordf &a2) const { 
	if ((a1[1] > 0.5 and a2[1] > 0.5) or (a1[1] < 0.5 or a2[1] < 0.5)) {
		FloatType angleDiff = std::fabs(a1[0] - a2[0]);
		return std::min(angleDiff, 2.0*M_PI - angleDiff);
	}

	return std::numeric_limits<FloatType>::infinity();
}
}