#include "PartitionAgent.hpp"
#include "PartitionNode.hpp"
#include "BeliefNode.hpp"
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>

namespace oppt {
PartitionAgent::PartitionAgent(RobotEnvironment *robotEnvironment,
                     BeliefNode *associatedBeliefNode,
                     const ProblemEnvironmentOptions *options,
                     const VectorFloat *lowerActionBound,
                     const VectorFloat *upperActionBound,
                     const FloatType &rootDiameter,
                     const DistanceMeasure *distanceMeasure,
                     const DiameterEstimator *diameterEstimator):	
	robotEnvironment_(robotEnvironment),
	associatedBeliefNode_(associatedBeliefNode),
	uniformDistribution_(new std::uniform_real_distribution<FloatType>(0.0, 1.0)),
	options_(options),
	rootDiameter_(rootDiameter),
	distanceMeasure_(distanceMeasure),
	diameterEstimator_(diameterEstimator) {
	initPartitionTree_(lowerActionBound, upperActionBound);
}

PartitionAgent::~PartitionAgent() {
	if (partitionTree_ != nullptr)
		delete partitionTree_;
}

void PartitionAgent::initPartitionTree_(const VectorFloat *lowerActionBound, const VectorFloat *upperActionBound) {
	const ADVTOptions * advtOptions = static_cast<const ADVTOptions *>(options_);
	partitionNodeTree_ = std::unique_ptr<Tree>(new Tree);
	partitionTree_ = new Tree;
	partitionNodeTree_->initRoot<PartitionNode>();
	auto root = partitionNodeTree_->getRoot()->as<PartitionNode>();
	root->setDepth(0.0);

	// Sample action uniformly at random
	VectorFloat randomActionVec(lowerActionBound->size(), 0.0);
	//std::shared_ptr<Action> randomAction(new VectorFloat(lowerActionBound->size(), 0.0));
	auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
	for (size_t i = 0; i != lowerActionBound->size(); ++i) {
		std::uniform_real_distribution<FloatType> dist((*lowerActionBound)[i], (*upperActionBound)[i]);
		randomActionVec[i] = dist(*randomEngine);
		//randomAction[i] = dist(*randomEngine);
	}

	//std::shared_ptr<Action> randomAction(new VectorAction(randomActionVec));
	std::shared_ptr<Action> randomAction = nullptr;
	//std::shared_ptr<Action> randomAction(new VectorAction(VectorFloat(lowerActionBound->size(), 0.0)));

	TreeElementPtr initPartition = nullptr;
	if (advtOptions->partitioningMode == "RECTANGLE") {
		randomAction = std::shared_ptr<Action>(new VectorAction(randomActionVec));
		initPartition = TreeElementPtr(new RectanglePartition(nullptr, randomAction, *lowerActionBound, *upperActionBound, advtOptions));
	} else if (advtOptions->partitioningMode == "VORONOI") {
		randomAction = std::shared_ptr<Action>(new VectorAction(randomActionVec));
		initPartition = TreeElementPtr(new VoronoiPartition(randomAction,
		                               nullptr,
		                               lowerActionBound,
		                               upperActionBound,
		                               rootDiameter_,
		                               advtOptions));
	} else if (advtOptions->partitioningMode == "VDP") {
		rootDiameter_ = 2.0 * M_PI;
		randomActionVec[1] = randomActionVec[1] > 0.5 ? 1.0 : 0.0;
		randomAction = std::shared_ptr<Action>(new VectorAction(randomActionVec));
		initPartition = TreeElementPtr(new VDPPartition(randomAction,
		                               nullptr,
		                               advtOptions));
	} else if (advtOptions->partitioningMode == "LINE_SEGMENT") {
		rootDiameter_ = (*upperActionBound)[0] - (*lowerActionBound)[0];
		randomAction = std::shared_ptr<Action>(new VectorAction(randomActionVec));
		initPartition = TreeElementPtr(new LineSegmentPartition(randomAction,
		                               nullptr,
		                               (*lowerActionBound)[0],
		                               (*upperActionBound)[0],
		                               advtOptions));
	} else {
		ERROR("Invalid partitioningMode. Valid options are 'RECTANGLE', 'VORONOI', 'VDP' and 'LINE_SEGMENT'");
	}

	TreeElement *initPartitionPtr = initPartition.get();
	initPartitionPtr->as<Partition>()->diameter_ = rootDiameter_;
	partitionTree_->updateRoot(std::move(initPartition));

	root->setPartition(initPartitionPtr);

	Action *action = initPartitionPtr->as<Partition>()->sampleAction(robotEnvironment_->getRobot()->getRandomEngine().get());


	//ActionUniquePtr newAction(new VectorAction(initPartitionPtr->as<Partition>()->sampleAction(robotEnvironment_->getRobot()->getRandomEngine().get())));
	//auto action = newAction.get();
	root->as<PartitionNode>()->setAction(action);
	//allActions_.push_back(std::move(newAction));
	leafNodes_.push_back(root);
	//splitNode(root, ADVTOptions->explorationFactor, ADVTOptions->explorationFactorDiameter);

}

void PartitionAgent::print() {
	auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
	FloatType splitExplorationFactor = static_cast<const ADVTOptions *>(options_)->splitExplorationFactor;
	cout << "splitExplorationFactor: " << splitExplorationFactor << endl;
	std::sort(leafNodes_.begin(), leafNodes_.end(), [](const TreeElement * a1, const TreeElement * a2) {
		return a1->as<PartitionNode>()->getAverageReward() < a2->as<PartitionNode>()->getAverageReward();
	});

	cout << "NUM LEAVES: " << leafNodes_.size() << endl;
	FloatType totalNumVisitsLeafes = std::accumulate(leafNodes_.begin(), leafNodes_.end(), 0.0, [](const FloatType & sum, const TreeElement * leafNode) {
		return sum + leafNode->as<PartitionNode>()->getNumVisits();
	});
	cout << "TOTAL NUM VISITS LEAFES: " << totalNumVisitsLeafes << endl;

	for (auto &leaf : leafNodes_) {
		auto leafNode = leaf->as<PartitionNode>();
		auto action = leafNode->getAction();
		auto partition = leafNode->getPartition();
		auto numVisits = leafNode->getNumVisits();
		FloatType diam = partition->as<Partition>()->getDiameter(diameterEstimator_, distanceMeasure_, randomEngine) / rootDiameter_;
		if (action) {
			cout << "a: " << *(action) << ", meanQ: " << leafNode->getAverageReward() << ", numVisits: " << numVisits;
			cout << ", depth: " << leafNode->getDepth() << ", diameter=" << diam <<
			     ", l=" << splitExplorationFactor * numVisits << ", r=" << 1.0 / (diam * diam) << endl;
		} else {
			cout << "a: NONE" << ", meanQ: " << leafNode->getAverageReward() << ", numVisits: " <<
			     leafNode->getNumVisits() << endl;
		}
	}
}

TreeElement *PartitionAgent::selectAction() {
	FloatType logN = log(((FloatType)(associatedBeliefNode_->getTotalVisitCount())));
	FloatType maxUValue = -std::numeric_limits<FloatType>::infinity();
	int maxLeaf = -1;
	for (auto i = 0; i != leafNodes_.size(); ++i) {
		FloatType uValue = leafNodes_[i]->as<PartitionNode>()->getUValue(logN,
		                   static_cast<const ADVTOptions *>(options_)->explorationFactor,
		                   static_cast<const ADVTOptions *>(options_)->explorationFactorDiameter,
		                   rootDiameter_,
		                   diameterEstimator_,
		                   distanceMeasure_,
		                   robotEnvironment_->getRobot()->getRandomEngine().get());
		if (uValue > maxUValue) {
			maxUValue = uValue;
			maxLeaf = i;
		}
	}

	return leafNodes_[maxLeaf];	
}

void PartitionAgent::splitNode(TreeElement *node, const FloatType &explorationFactor, const FloatType &explorationFactorDiameter) {
	auto nodeDepth = node->as<PartitionNode>()->getDepth();
	auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
	auto childPartitions =
	    node->as<PartitionNode>()->getPartition()->as<Partition>()->split(diameterEstimator_,
	            distanceMeasure_,
	            randomEngine);
	auto childPartition1 = childPartitions.first;
	auto childPartition2 = childPartitions.second;

	TreeElementPtr child1(new PartitionNode(node));
	child1->as<PartitionNode>()->setPartition(childPartition1);
	child1->as<PartitionNode>()->setDepth(nodeDepth + 1.0);
	TreeElementPtr child2(new PartitionNode(node));
	child2->as<PartitionNode>()->setPartition(childPartition2);
	child2->as<PartitionNode>()->setDepth(nodeDepth + 1.0);

	TreeElement *child1Ptr = child1.get();
	TreeElement *child2Ptr = child2.get();
	node->addChild(std::move(child1));
	node->addChild(std::move(child2));

	for (size_t i = 0; i != leafNodes_.size(); ++i) {
		if (leafNodes_[i] == node) {
			leafNodes_.erase(leafNodes_.begin() + i);
			break;
		}
	}

	leafNodes_.push_back(child1Ptr);
	leafNodes_.push_back(child2Ptr);

	// New action for the child sibling node
	Action *actionChild = childPartition2->as<Partition>()->sampleAction(robotEnvironment_->getRobot()->getRandomEngine().get());

	child1Ptr->as<PartitionNode>()->setAction(node->as<PartitionNode>()->getAction());
	child1Ptr->as<PartitionNode>()->averageReward_ = node->as<PartitionNode>()->averageReward_;
	child1Ptr->as<PartitionNode>()->numVisits_ = node->as<PartitionNode>()->numVisits_;
	child2Ptr->as<PartitionNode>()->setAction(actionChild);
	FloatType numVisitsBelief = ((FloatType)(associatedBeliefNode_->getTotalVisitCount()));
	
	FloatType diam =
	    child1Ptr->as<PartitionNode>()->getPartition()->as<Partition>()->getDiameter(diameterEstimator_, distanceMeasure_, randomEngine) / rootDiameter_;
	bool split = static_cast<const ADVTOptions *>(options_)->splitExplorationFactor * child1Ptr->as<PartitionNode>()->getNumVisits() >= 1.0 / (diam * diam);

	if (split and child1Ptr->as<PartitionNode>()->getPartition()->as<Partition>()->isSplittable(rootDiameter_)) {
		splitNode(child1Ptr, explorationFactor, explorationFactorDiameter);
	}	
}

std::vector<TreeElement *> PartitionAgent::getLeafNodes() const {
	return leafNodes_;
}

TreeElement *PartitionAgent::getBestAction() {
	return *(std::max_element(leafNodes_.begin(), leafNodes_.end(), [](const TreeElement * a1, const TreeElement * a2) {
		return a1->as<PartitionNode>()->getAverageReward() < a2->as<PartitionNode>()->getAverageReward();
	}));
}

void PartitionAgent::updateReward(TreeElement *node,
                             const FloatType &reward) {
	const ADVTOptions *advtOptions = static_cast<const ADVTOptions *>(options_);
	node->as<PartitionNode>()->updateReward(reward);

	auto partition = node->as<PartitionNode>()->getPartition();
	FloatType diam =
	    partition->as<Partition>()->getDiameter(diameterEstimator_, distanceMeasure_, robotEnvironment_->getRobot()->getRandomEngine().get()) /
	    rootDiameter_;
	bool splittable1 = partition->as<Partition>()->isSplittable(rootDiameter_);
	bool splittable2 = advtOptions->splitExplorationFactor * node->as<PartitionNode>()->getNumVisits() >= 1.0 / (diam * diam);
	if (splittable1 and splittable2) {
		splitNode(node, advtOptions->explorationFactor, advtOptions->explorationFactorDiameter);
	}
}

}