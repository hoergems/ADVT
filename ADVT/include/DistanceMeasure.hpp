#ifndef _DISTANCE_METRIC_HPP_
#define _DISTANCE_METRIC_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {
class DistanceMeasure {
public:
	virtual FloatType operator()(const VectorFloat &a1, const VectorFloat &a2) const = 0;

	virtual FloatType operator()(const Vectordf &a1, const Vectordf &a2) const = 0;
};

class EuclideanDistanceMeasure: public DistanceMeasure {
public:
	virtual FloatType operator()(const VectorFloat &a1, const VectorFloat &a2) const override;

	virtual FloatType operator()(const Vectordf &a1, const Vectordf &a2) const override;
};

class VDPMeasure: public DistanceMeasure {
public:
	virtual FloatType operator()(const VectorFloat &a1, const VectorFloat &a2) const override;

	virtual FloatType operator()(const Vectordf &a1, const Vectordf &a2) const override;
};

}

#endif