#include "Partition.hpp"
#include "PartitionNode.hpp"
#include <oppt/global.hpp>
#include <oppt/robotHeaders/Action.hpp>

namespace oppt {
VDPPartition::VDPPartition(const std::shared_ptr<Action> &action,
                           TreeElement *const parentPartition,
                           const ProblemEnvironmentOptions *options):
	Partition(parentPartition, action, options) {

}

const FloatType VDPPartition::getL() const {
	return l;
}

const FloatType VDPPartition::getR() const {
	return r;
}

Action *VDPPartition::sampleAction(RandomEngine *randomEngine) const {
	return action_.get();
}

std::pair<TreeElement*, TreeElement*> VDPPartition::split(const DiameterEstimator *diameterEstimator,
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

	TreeElementPtr childPartition1 = nullptr;
	TreeElementPtr childPartition2 = nullptr;
	if (getParent() == nullptr) {
		childPartition1 = TreeElementPtr (new VDPPartition(action_,
		                                  this,
		                                  options_));

		VectorFloat action2Vec = action_->as<VectorAction>()->asVector();
		action2Vec[1] = action2Vec[1] > 0.5 ? 0.0 : 1.0;
		std::shared_ptr<Action> action2(new VectorAction(action2Vec));		
		childPartition2 = TreeElementPtr(new VDPPartition(action2,
		                                 this,
		                                 options_));

	} else {
		childPartition1 = TreeElementPtr (new VDPPartition(action_,
		                                  this,
		                                  options_));
		std::shared_ptr<Action> newAction(new VectorAction(sampleUniformlyFromPartition_(randomEngine)));
		childPartition2 = TreeElementPtr(new VDPPartition(newAction,
		                                 this,
		                                 options_));
	}

	std::pair<TreeElement*, TreeElement*> childPartitions = {addChild(std::move(childPartition1)), addChild(std::move(childPartition2))};
	return childPartitions;
}

TreeElement *VDPPartition::getSiblingPartition() const {
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

VectorFloat VDPPartition::sampleUniformlyFromPartition_(RandomEngine *randomEngine) const {
	std::uniform_real_distribution<FloatType> d(l, r);
	return VectorFloat({d(*randomEngine), action_->as<VectorAction>()->asVector()[1]});
}

/**void VDPPartition::computeDiameter(RandomEngine *randomEngine) const {
	if (getParent() == nullptr or getParent()->getParent() == nullptr) {
		diam_ = 2.0 * M_PI;
		return;
	}

	FloatType parentL = getParent()->as<VDPPartition>()->getL();
	FloatType parentR = getParent()->as<VDPPartition>()->getR();


	VectorFloat siblingAction = getSiblingPartition()->as<Partition>()->sampleAction(nullptr);
	if (action_[0] < siblingAction[0]) {
		l = parentL;
		r = action_[0] + (siblingAction[0] - action_[0]) / 2.0;
	} else {
		r = parentR;
		l = siblingAction[0] + (action_[0] - siblingAction[0]) / 2.0;
	}

	diameter_ = r - l;
}*/

FloatType VDPPartition::getDiameter(const DiameterEstimator *diameterEstimator,
                                    const DistanceMeasure *distanceMeasure,
                                    RandomEngine *randomEngine) const {
	if (std::isnan(diameter_)) {
		if (getParent() == nullptr or getParent()->getParent() == nullptr) {
			diameter_ = 2.0 * M_PI;
		} else {

			FloatType parentL = getParent()->as<VDPPartition>()->getL();
			FloatType parentR = getParent()->as<VDPPartition>()->getR();

			VectorFloat action = action_->as<VectorAction>()->asVector();
			VectorFloat siblingAction = getSiblingPartition()->as<Partition>()->sampleAction(nullptr)->as<VectorAction>()->asVector();
			if (action[0] < siblingAction[0]) {
				l = parentL;
				r = action[0] + (siblingAction[0] - action[0]) / 2.0;
			} else {
				r = parentR;
				l = siblingAction[0] + (action[0] - siblingAction[0]) / 2.0;
			}

			diameter_ = r - l;
		}

	}

	return diameter_;
}

bool VDPPartition::isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const {
	VectorFloat thisAction = action_->as<VectorAction>()->asVector();
	if ((thisAction[1] < 0.5 and action[1] > 0.5) or (thisAction[1] > 0.5 and action[1] < 0.5))
		return false;

	if (action[0] < l or action[0] > r)
		return false;
	return true;

}

bool VDPPartition::isSplittable(const FloatType &rootDiameter) const {
	return true;
}

void VDPPartition::serializePartition(std::ofstream &os,
                                      const DiameterEstimator *diameterEstimator,
                                      const DistanceMeasure *distanceMeasure,
                                      RandomEngine *randomEngine) {
	ERROR("Not implemented");
}
}