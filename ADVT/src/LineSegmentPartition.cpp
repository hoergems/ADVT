#include "Partition.hpp"
#include "PartitionNode.hpp"
#include <oppt/global.hpp>
#include <oppt/robotHeaders/Action.hpp>

namespace oppt {
LineSegmentPartition::LineSegmentPartition(const std::shared_ptr<Action> &action,
        TreeElement *const parentPartition,
        const FloatType &l,
        const FloatType &r,
        const ProblemEnvironmentOptions *options):
	Partition(parentPartition, action, options),
	l_(l),
	r_(r) {

}

const FloatType LineSegmentPartition::getL() const {
	return l_;
}

const FloatType LineSegmentPartition::getR() const {
	return r_;
}

Action *LineSegmentPartition::sampleAction(RandomEngine *randomEngine) const {
	return action_.get();
}

std::pair<TreeElement*, TreeElement*> LineSegmentPartition::split(const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) {
	if (getNumChildren() == 2) {
		auto ch = getChildren();
		auto ch1 = (*ch).get();
		ch++;
		auto ch2 = (*ch).get();
		std::pair<TreeElement*, TreeElement*> childPartitions = {ch1, ch2};
		return childPartitions;
	}

	if (std::isnan(diameter_)) {
		ERROR("WFT");
	}

	std::shared_ptr<Action> newAction(new VectorAction(sampleUniformlyFromPartition_(randomEngine)));
	TreeElementPtr childPartition1(new LineSegmentPartition(action_,
	                               this,
	                               0.0,
	                               0.0,
	                               options_));
	TreeElementPtr childPartition2(new LineSegmentPartition(newAction,
	                               this,
	                               0.0,
	                               0.0,
	                               options_));

	std::pair<TreeElement*, TreeElement*> childPartitions = {addChild(std::move(childPartition1)), addChild(std::move(childPartition2))};
	return childPartitions;
}

FloatType LineSegmentPartition::getDiameter(const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) const {
	if (std::isnan(diameter_)) {
		if (getParent() == nullptr) {
			diameter_ = r_ - l_;
		} else {
			FloatType parentL = getParent()->as<VDPPartition>()->getL();
			FloatType parentR = getParent()->as<VDPPartition>()->getR();
			VectorFloat action = action_->as<VectorAction>()->asVector();
			VectorFloat siblingAction = getSiblingPartition()->as<Partition>()->sampleAction(nullptr)->as<VectorAction>()->asVector();
			if (action[0] < siblingAction[0]) {
				l_ = parentL;
				r_ = action[0] + (siblingAction[0] - action[0]) / 2.0;
			} else {
				r_ = parentR;
				l_ = siblingAction[0] + (action[0] - siblingAction[0]) / 2.0;
			}

			diameter_ = r_ - l_;
		}
	}

	return diameter_;

}

bool LineSegmentPartition::isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const {
	if (action[0] < l_ or action[0] > r_)
		return false;
	return true;
}

bool LineSegmentPartition::isSplittable(const FloatType &rootDiameter) const {
	return true;
}

VectorFloat LineSegmentPartition::sampleUniformlyFromPartition_(RandomEngine *randomEngine) const {
	if (sampleDistribution_ == nullptr) {
		sampleDistribution_ = std::make_unique<std::uniform_real_distribution<FloatType>>(l_, r_);
	}

	return VectorFloat({(*(sampleDistribution_.get()))(*randomEngine)});
}

TreeElement *LineSegmentPartition::getSiblingPartition() const {
	if (getParent() == nullptr)
		return nullptr;

	auto parentChildPartitions = parentElement_->getChildren();
	auto ch1 = (*parentChildPartitions).get();
	parentChildPartitions++;
	auto ch2 = (*parentChildPartitions).get();

	if (ch1 == this) {
		return ch2;
	} else if (ch2 == this) {
		return ch1;
	} else {
		ERROR("HUH");
		getchar();
	}
}

void LineSegmentPartition::serializePartition(std::ofstream &os,
        const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) {
	ERROR("Not implemented");
}

}