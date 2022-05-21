#include "PartitionNode.hpp"
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>

namespace oppt {
PartitionNode::PartitionNode(TreeElement *const parentElement):
	TreeElement(parentElement) {

}

void PartitionNode::print() const {

}

void PartitionNode::setDepth(const FloatType &depth) {
	depth_ = depth;
}

FloatType PartitionNode::getDepth() const {
	return depth_;
}

FloatType PartitionNode::getUValue(const FloatType &logN,
                              const FloatType &explorationFactor,
                              const FloatType &explorationFactorDiameter,
                              const FloatType &rootDiameter,
                              const DiameterEstimator *diameterEstimator,
                              const DistanceMeasure *distanceMeasure, 
                              RandomEngine *randomEngine) const {
	if (numVisits_ < 1)
		return std::numeric_limits<FloatType>::infinity();
	if (std::isnan(termDiam_))
		termDiam_ = (explorationFactorDiameter / rootDiameter) * partition_->as<Partition>()->getDiameter(diameterEstimator, distanceMeasure, randomEngine);
	return averageReward_ + explorationFactor * sqrt(logN / numVisits_) + termDiam_;

	       
}

FloatType PartitionNode::getAverageReward() const {
	return averageReward_;
}

void PartitionNode::setPartition(TreeElement *const partition) {
	partition_ = partition;
}

void PartitionNode::updateReward(const FloatType &reward) {
	if (numVisits_ < 0.5) {
		averageReward_ = 0.0;
	}

	numVisits_ += 1.0;
	averageReward_ = averageReward_ + (1.0 / numVisits_) * (reward - averageReward_);
	//averageReward_ = (1.0 - 1.0 / numVisits_) * averageReward_ + reward / numVisits_;
}

TreeElement *PartitionNode::getPartition() const {
	if (partition_)
		return partition_;
	return nullptr;
}

void PartitionNode::setAction(Action *action) {
	action_ = action;
}

Action* PartitionNode::getAction() const {
	return action_;
}


FloatType PartitionNode::getNumVisits() const {
	return numVisits_;
}

}