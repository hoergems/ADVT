#ifndef _DIAMETER_ESTIMATOR_HPP_
#define _DIAMETER_ESTIMATOR_HPP_
#include <oppt/opptCore/core.hpp>
#include "DistanceMeasure.hpp"

namespace oppt {
class Partition;
class VoronoiPartition;

typedef std::vector<VectorFloat> BoundaryPoints;

class DiameterEstimator {
public:
	friend class Partition;
	friend class VoronoiPartition;
	DiameterEstimator();

	virtual ~DiameterEstimator() = default;

	_NO_COPY_BUT_MOVE(DiameterEstimator)

	virtual FloatType estimateDiameter(RandomEngine *randomEngine,
	                                   const Partition *partition,
	                                   const DistanceMeasure *distanceMeasure,
	                                   const unsigned int &numBoundaryPoints,
	                                   BoundaryPoints &currentBoundaryPoints,
	                                   const Vectordf &voronoiCenter,
	                                   const FloatType &rootDiameter) const;

	virtual std::vector<VectorFloat> sampleUniform(const unsigned int &numSamples,
			const Partition *partition,
			const DistanceMeasure *distanceMeasure,	        
	        RandomEngine *randomEngine,
	        const FloatType &rootDiameter) const;	

protected:
	virtual std::vector<Vectordf> samplePointPairsFromOutsideActionSpace(const unsigned int &numPoints,
	        const Vectordf &voronoiCenter,
	        RandomEngine *randomEngine,
	        const FloatType &rootDiameter) const;

	virtual Vectordf projectOntoBoundary(const Partition *partition,
	                                     const Vectordf &voronoiCenter,
	                                     const Vectordf &point, 
	                                     const DistanceMeasure *distanceMeasure) const;

protected:
	const Partition *partition_ = nullptr;

	std::unique_ptr<std::normal_distribution<FloatType>> normalDistribution_ = nullptr;

	std::unique_ptr<std::uniform_real_distribution<FloatType>> uniformDistribution_ = nullptr;
};
}

#endif