#ifndef _PARTITION_NODE_HPP_
#define _PARTITION_NODE_HPP_
#include <oppt/opptCore/core.hpp>
#include "TreeElement.hpp"
#include "Partition.hpp"

namespace solvers {
class HOOT;
}

namespace oppt {
class PartitionAgent;
class PartitionNode: public TreeElement {
public:
	PartitionNode(TreeElement *const parentElement);
	friend class solvers::HOOT;
	friend class PartitionAgent;

	virtual ~PartitionNode() = default;

	virtual void print() const override;

	//void split(RandomEngine *randomEngine);	

	FloatType getUValue(const FloatType &logN,
	                    const FloatType &explorationFactor,
	                    const FloatType &explorationFactorDiameter,
	                    const FloatType &rootDiameter,
	                    const DiameterEstimator *diameterEstimator,
	                    const DistanceMeasure *distanceMeasure, 
	                    RandomEngine *randomEngine) const;

	FloatType getAverageReward() const;

	void setDepth(const FloatType &depth);

	FloatType getDepth() const;

	void updateReward(const FloatType &reward);

	//void updateUValue(const FloatType &n, const FloatType &explorationFactor, const FloatType &explorationFactorDiameter, const FloatType &rootDiameter);	

	void setPartition(TreeElement *const partition);

	TreeElement *getPartition() const;

	void setAction(Action *action);

	Action* getAction() const;

	FloatType getNumVisits() const;

	//void overrideNumVisits(const FloatType &numVisits);

	//void overrideAverageReward(const FloatType &averageReward);

private:
	FloatType uValue_ = std::numeric_limits<FloatType>::infinity();

	FloatType averageReward_ = -std::numeric_limits<FloatType>::infinity();

	FloatType numVisits_ = 0.0;

	FloatType depth_ = 0.0;

	Action *action_ = nullptr;

	TreeElement *partition_ = nullptr;

	mutable FloatType termDiam_ = std::numeric_limits<FloatType>::quiet_NaN();
};
}

#endif