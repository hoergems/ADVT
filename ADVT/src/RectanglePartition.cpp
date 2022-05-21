#include "Partition.hpp"
#include "PartitionNode.hpp"
#include <oppt/global.hpp>
#include <oppt/robotHeaders/Action.hpp>

namespace oppt {
RectanglePartition::RectanglePartition(TreeElement *const parentPartition,
                                       const std::shared_ptr<Action> &action,
                                       const VectorFloat &lowerBound,
                                       const VectorFloat &upperBound,
                                       const ProblemEnvironmentOptions *options):
	Partition(parentPartition, action, options),
	lower_(lowerBound),
	upper_(upperBound) {

}

Action *RectanglePartition::sampleAction(RandomEngine *randomEngine) const {
	return action_.get();
}

VectorFloat RectanglePartition::sampleUniformlyFromPartition_(RandomEngine *randomEngine) const {
	VectorFloat action(lower_.size(), 0.0);
	for (size_t i = 0; i != lower_.size(); ++i) {
		std::uniform_real_distribution<FloatType> dist(lower_[i], upper_[i]);
		action[i] = dist(*randomEngine);
	}

	return action;
}

std::pair<TreeElement*, TreeElement*> RectanglePartition::split(const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) {
	if (getNumChildren() > 0) {
		auto ch = getChildren();
		auto ch1 = (*ch).get();
		ch++;
		auto ch2 = (*ch).get();
		std::pair<TreeElement*, TreeElement*> childPartitions = {ch1, ch2};
		return childPartitions;
	}
	unsigned int longestEdge = 0;
	FloatType maxRange = -std::numeric_limits<FloatType>::infinity();
	for (size_t i = 0; i != lower_.size(); ++i) {
		FloatType range = upper_[i] - lower_[i];
		if (range > maxRange) {
			maxRange = range;
			longestEdge = i;
		}
	}

	TreeElementPtr actionRange1(new RectanglePartition(this, action_, lower_, upper_, options_));
	TreeElementPtr actionRange2(new RectanglePartition(this, action_, lower_, upper_, options_));


	actionRange1->as<RectanglePartition>()->upper_[longestEdge] = lower_[longestEdge] + ((upper_[longestEdge] - lower_[longestEdge]) / 2.0);
	actionRange2->as<RectanglePartition>()->lower_[longestEdge] = actionRange1->as<RectanglePartition>()->upper_[longestEdge];

	std::pair<TreeElement*, TreeElement*> actionRanges;
	//if (actionRange1->as<RectanglePartition>()->isInPartition(distanceMeasure, *(action_.get()))) {
	if (actionRange1->as<RectanglePartition>()->isInPartition(distanceMeasure, action_->as<VectorAction>()->asVector())) {
		actionRange1->as<RectanglePartition>()->action_ = action_;
		actionRange2->as<RectanglePartition>()->action_ =
		    std::shared_ptr<Action>(new VectorAction(actionRange2->as<RectanglePartition>()->sampleUniformlyFromPartition_(randomEngine)));
		//actionRange2->as<RectanglePartition>()->action_ = actionRange2->as<RectanglePartition>()->sampleUniformlyFromPartition_(randomEngine);
		actionRanges = {addChild(std::move(actionRange1)), addChild(std::move(actionRange2))};
	} else {
		actionRange2->as<RectanglePartition>()->action_ = action_;
		actionRange1->as<RectanglePartition>()->action_ =
		    std::shared_ptr<Action>(new VectorAction(actionRange1->as<RectanglePartition>()->sampleUniformlyFromPartition_(randomEngine)));
		actionRanges = {addChild(std::move(actionRange2)), addChild(std::move(actionRange1))};
	}

	return actionRanges;
}

bool RectanglePartition::isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const {
	for (size_t i = 0; i != action.size(); ++i) {
		if (action[i] < lower_[i] or action[i] > upper_[i])
			return false;
	}

	return true;
}

bool RectanglePartition::isSplittable(const FloatType &rootDiameter) const {
	return true;
	return diameter_ < 1e-5 ? false : true;
}

FloatType RectanglePartition::getDiameter(const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) const {
	if (std::isnan(diameter_)) {
		//printVector(upper_, "upper");
		//printVector(lower_, "lower");
		diameter_ = (*distanceMeasure)(upper_, lower_);
		//cout << "diam: " << diameter_ << endl;
	}
	return diameter_;
}

void RectanglePartition::serializePartition(std::ofstream &os,
        const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) {
	ERROR("Not implemented");
}
}