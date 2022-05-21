#include "Partition.hpp"
#include "PartitionNode.hpp"
#include <oppt/global.hpp>
#include <oppt/robotHeaders/Action.hpp>

namespace oppt {
VoronoiPartition::VoronoiPartition(const std::shared_ptr<Action> &action,
                                   TreeElement *const parentPartition,
                                   const VectorFloat *lowerActionBounds,
                                   const VectorFloat *upperActionBounds,
                                   const FloatType &rootDiameter,
                                   const ProblemEnvironmentOptions *options):
	Partition(parentPartition, action, options),
	lowerActionBounds_(lowerActionBounds),
	upperActionBounds_(upperActionBounds),
	rootDiameter_(rootDiameter) {

}

std::pair<TreeElement*, TreeElement*> VoronoiPartition::split(const DiameterEstimator *diameterEstimator,
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

	std::shared_ptr<Action> newAction(new VectorAction(sampleUniformlyFromPartition_(diameterEstimator, distanceMeasure, randomEngine)));
	//VectorFloat newAction = sampleUniformlyFromPartition_(diameterEstimator, distanceMeasure, randomEngine);
	TreeElementPtr childPartition1(new VoronoiPartition(action_,
	                               this,
	                               lowerActionBounds_,
	                               upperActionBounds_,
	                               rootDiameter_,
	                               options_));
	TreeElementPtr childPartition2(new VoronoiPartition(newAction,
	                               this,
	                               lowerActionBounds_,
	                               upperActionBounds_,
	                               rootDiameter_,
	                               options_));

	//////////////////////////////////////////
	auto options = static_cast<const ADVTOptions *>(options_);
	BoundaryPoints bp1, bp2;
	bp1.reserve(options->numDiameterSamples);
	bp2.reserve(options->numDiameterSamples);
	for (size_t i = 0; i != boundaryPoints_.size(); ++i) {
		if ((*distanceMeasure)(boundaryPoints_[i], action_->as<VectorAction>()->asVector()) <= (*distanceMeasure)(boundaryPoints_[i], newAction->as<VectorAction>()->asVector())) {
			bp1.push_back(boundaryPoints_[i]);
		} else {
			bp2.push_back(boundaryPoints_[i]);
		}
		//(*boundaryPoints)[i];
	}

	childPartition1->as<VoronoiPartition>()->boundaryPoints_ = bp1;
	childPartition2->as<VoronoiPartition>()->boundaryPoints_ = bp2;

	//////////////////////////////////////////

	std::pair<TreeElement*, TreeElement*> childPartitions = {addChild(std::move(childPartition1)), addChild(std::move(childPartition2))};
	return childPartitions;
}

/**void VoronoiPartition::computeDiameter(RandomEngine *randomEngine) const {
	if (std::isnan(diameter_)) {
		diameter_ = diameterEstimator_->estimateDiameter(randomEngine, static_cast<const ADVTOptions *>(options_)->numDiameterSamples, rootDiameter_);
	}
}*/

FloatType VoronoiPartition::getDiameter(const DiameterEstimator *diameterEstimator, const DistanceMeasure *distanceMeasure, RandomEngine *randomEngine) const {
	if (std::isnan(diameter_)) {
		diameter_ = diameterEstimator->estimateDiameter(randomEngine,
		            this,
		            distanceMeasure,
		            static_cast<const ADVTOptions *>(options_)->numDiameterSamples,
		            boundaryPoints_,
		            toEigenVec(sampleAction(randomEngine)->as<VectorAction>()->asVector()),
		            rootDiameter_);
	}

	return diameter_;
}

Action *VoronoiPartition::sampleAction(RandomEngine *randomEngine) const {
	return action_.get();
}

bool VoronoiPartition::isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const {
	if (parentElement_ == nullptr) {
		for (size_t i = 0; i != action.size(); ++i) {
			if (action[i] < (*lowerActionBounds_)[i] or action[i] > (*upperActionBounds_)[i])
				return false;
		}

		return true;
	}

	auto parentChildPartitions = parentElement_->getChildren();
	auto ch1 = (*parentChildPartitions).get();
	parentChildPartitions++;
	auto ch2 = (*parentChildPartitions).get();

	TreeElement *siblingPartition = nullptr;

	if (ch1 == this) {
		siblingPartition = ch2;
	} else if (ch2 == this) {
		siblingPartition = ch1;
	} else {
		WARNING("HUH");
		getchar();
	}

	if ((*distanceMeasure)(siblingPartition->as<Partition>()->sampleAction(nullptr)->as<VectorAction>()->asVector(), action) <
	        (*distanceMeasure)(action_->as<VectorAction>()->asVector(), action))
		return false;

	return parentElement_->as<Partition>()->isInPartition(distanceMeasure, action);
}

bool VoronoiPartition::isSplittable(const FloatType &rootDiameter) const {
	//return diameter_ / rootDiameter > minimumSplittingDiam_;
	return true;
	//return diameter_ < 1e-5 ? false : true;
}

VectorFloat VoronoiPartition::sampleUniformlyFromPartition_(const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) const {
	return diameterEstimator->sampleUniform(1, this, distanceMeasure, randomEngine, rootDiameter_)[0];
}

void VoronoiPartition::serializePartition(std::ofstream &os, const DiameterEstimator *diameterEstimator,
        const DistanceMeasure *distanceMeasure,
        RandomEngine *randomEngine) {
	os << "NODE_BEGIN ";
	os << "id: " << getID() << ", ";
	if (parentElement_ == nullptr) {
		os << "parent: None, " ;
	} else {
		os << "parent: " << parentElement_->as<Partition>()->getID() << ", ";
	}

	if (getNumChildren() == 2) {
		auto children = getChildren();
		auto child1 = (*children).get();
		children++;
		auto child2 = (*children).get();
		os << "children: " << child1->as<Partition>()->getID() << " " << child2->as<Partition>()->getID() << ", ";
	} else {
		os << "children: None, ";
	}

	VectorFloat action = action_->as<VectorAction>()->asVector();
	os << "action: ";
	for (size_t i = 0; i != action.size(); ++i) {
		if (i == action.size() - 1) {
			os << action[i];
		} else {
			os << action[i] << " ";
		}
	}
	os << ", ";
	os << "diameter: " << getDiameter(diameterEstimator, distanceMeasure, randomEngine) << ", ";
	os << "NODE_END" << endl;
}
}