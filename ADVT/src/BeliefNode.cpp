#include "BeliefNode.hpp"
#include "BeliefNodeData.hpp"
#include "PartitionNode.hpp"

namespace oppt {

BeliefNode::BeliefNode(TreeElement *const parentElement):
	TreeElement(parentElement) {

}

bool BeliefNode::isNew() const {
	return isNew_;
}

void BeliefNode::initialize(RobotEnvironment *robotEnvironment,
                            ObservationComparator observationComparator,
                            const ProblemEnvironmentOptions *options,
                            const VectorFloat *lowerActionBound,
                            const VectorFloat *upperActionBound,
                            const FloatType &rootDiameter,
                            const DistanceMeasure *distanceMeasure,
                            const DiameterEstimator *diameterEstimator) {
	partitionAgent_ = std::unique_ptr<PartitionAgent>(new PartitionAgent(robotEnvironment,
	         this,
	         options,
	         lowerActionBound,
	         upperActionBound,
	         rootDiameter,
	         distanceMeasure,
	         diameterEstimator));
	//partitionAgent_ = std::unique_ptr<Agent>(new GaussianAgent(robotEnvironment, this));
	treeElementData_ = std::unique_ptr<TreeElementData>(new BeliefNodeData(this, robotEnvironment));
	isNew_ = false;
	observationComparator_ = observationComparator;
	cachedValue_ = 0.0;
	totalVisitCount_ = 0;
	children_ = VectorTreeElement();
}

TreeElement* BeliefNode::getActionEdge(const Action *action) const {
	for (auto &child : children_) {
		if (child->as<ActionEdge>()->getAction() == action)
			return child.get();
	}

	return nullptr;
}


void BeliefNode::print() const {
	partitionAgent_->print();
	/**std::vector<std::pair<FloatType, std::string>> actionStats;
	for (size_t i = 0; i != getNumChildren(); ++i) {
		auto actionEdge = children_[i]->as<ActionEdge>();
		bool active = actionEdge->getData()->as<ActionEdgeData>()->isActive();
		if (active) {
			std::stringstream s;
			s << *(actionEdge->getAction());
			std::string actionString = "action " + s.str();
			FloatType qValue = actionEdge->as<ActionEdge>()->getMeanQ();
			auto lowerUpperConfidenceBounds = actionEdge->as<ActionEdge>()->getLowerUpperConfidenceBounds();
			actionString += ", numVisits=" + std::to_string(actionEdge->as<ActionEdge>()->getNumVisits());
			actionString += ", meanQ=" + std::to_string(qValue);
			actionString += ", numObservations: " + std::to_string(actionEdge->getNumChildren());
			actionString += ", confidenceBounds: " + std::to_string((*lowerUpperConfidenceBounds).second - qValue);
			actionString += " (" + std::to_string((*lowerUpperConfidenceBounds).first) + ", " + std::to_string((*lowerUpperConfidenceBounds).second) + ")";
			actionString += ", active=" + std::to_string(active);
			actionString += ", " + std::to_string((FloatType)(actionEdge->as<ActionEdge>()->getNumVisits()) / ((FloatType)(totalVisitCount_)));

			std::pair<FloatType, std::string> stat = {actionEdge->as<ActionEdge>()->getMeanQ(), actionString};
			actionStats.push_back(stat);
		}
	}

	std::sort(actionStats.begin(), actionStats.end(), [](const std::pair<FloatType, std::string> &p1, const std::pair<FloatType, std::string> &p2) {
		return p1.first > p2.first;
	});

	for (auto &stat : actionStats) {
		cout << stat.second << endl;
	}*/
}

void BeliefNode::updateAgent(TreeElement *node,
                             const FloatType &reward) {
	partitionAgent_->updateReward(node, reward);
}


RobotStateSharedPtr BeliefNode::sampleParticle() const {
	return static_cast<BeliefNodeData *>(treeElementData_.get())->sampleParticle();
}

TreeElement * BeliefNode::getBestAction() const {
	return partitionAgent_->getBestAction();
}

std::pair<TreeElement *, TreeElement *> const BeliefNode::selectAction() {
	auto actionNode = partitionAgent_->selectAction();
	auto action = actionNode->as<PartitionNode>()->getAction();
	for (auto &child : children_) {
		if (child->as<ActionEdge>()->getAction() == action) {
			return std::pair<TreeElement *, TreeElement *>(actionNode, child.get());
			//return actionNode;
		}
	}

	TreeElementPtr newActionEdge(new ActionEdge(this, action));
	TreeElement *newActionEdgePtr = newActionEdge.get();
	newActionEdge->as<ActionEdge>()->setObservationComparator(observationComparator_);
	children_.push_back(std::move(newActionEdge));

	return std::pair<TreeElement *, TreeElement *>(actionNode, newActionEdgePtr);
	//return actionNode;
}

FloatType BeliefNode::recalculateValue() {
	cachedValue_ = -std::numeric_limits<FloatType>::infinity();
	auto bestAction = getBestAction();
	if (bestAction)
		cachedValue_ = bestAction->as<PartitionNode>()->getAverageReward();

	return cachedValue_;
}

FloatType BeliefNode::getCachedValue() {
	if (totalVisitCount_ <= 0)
		return 0.0;
	return cachedValue_;
}

long BeliefNode::getTotalVisitCount() const {
	return totalVisitCount_;
}

void BeliefNode::updateVisitCount(const long &visitCount) {
	totalVisitCount_ += visitCount;
	if (totalVisitCount_ < 0)
		totalVisitCount_ = 0;
}

PartitionAgent *BeliefNode::getPartitionAgent() const {
	return partitionAgent_.get();
}


}

