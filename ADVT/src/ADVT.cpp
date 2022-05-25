#include "ADVT.hpp"
#include "ADVTOptions.hpp"
#include "PartitionNode.hpp"
#include "BeliefNode.hpp"
#include "BeliefNodeData.hpp"
#include <oppt/robotHeaders/ActionSpaceDiscretizer.hpp>
#include "BoundingSphere.hpp"

namespace oppt {
extern ObservationComparator observationComparator;
}

namespace solvers {
ADVT::ADVT():
	Solver() {
	solverName_ = "ADVT";
}

void ADVT::setup() {
	checkOptions_();
	auto options = static_cast<const ADVTOptions *>(problemEnvironmentOptions_);
	oppt::boundingSphere::GenerateDiamFunctions::generate();

	// Generate distance measure
	std::string distanceMeasureStr = options->distanceMeasure;
	if (distanceMeasureStr == "euclidean") {
		distanceMeasure_ = std::make_unique<EuclideanDistanceMeasure>();
	} else if (distanceMeasureStr == "VDP") {
		distanceMeasure_ = std::make_unique<VDPMeasure>();
	} else {
		ERROR("Distance measure '" + distanceMeasureStr + "' not supported");
	}

	diameterEstimator_ = std::make_unique<DiameterEstimator>();
	///////////////////////
	// Compute the root diameter
	lowerActionBound_ = VectorFloat();
	upperActionBound_ = VectorFloat();
	robotPlanningEnvironment_->getRobot()->getActionSpace()->getActionLimits()->getLimits()->as<VectorLimitsContainer>()->get(lowerActionBound_, upperActionBound_);
	rootDiameter_ = math::euclideanDistance(lowerActionBound_, upperActionBound_);	

	reset();

	FloatType maxObservationDistance = options->maxObservationDistance;
	observationComparator_ = [maxObservationDistance](const Observation * observation, TreeElement * treeElement) {
		auto it = treeElement->getChildren();
		unsigned int numChildren = treeElement->getNumChildren();
		unsigned int idx = 0;		
		FloatType smallestDistance = maxObservationDistance;		
		TreeElement *closestObservationEdge = nullptr;
		while (true) {
			idx++;
			if (idx > numChildren)
				break;
			FloatType dist = (*it)->as<ObservationEdge>()->getObservation()->distanceTo(*observation);
			if (dist <= smallestDistance) {
				smallestDistance = dist;
				closestObservationEdge = (*it).get();
			}

			it++;
		}

		return closestObservationEdge;
	};

	particleFilter_ = std::make_unique<ParticleFilter>();
	propReq_ = PropagationRequestSharedPtr(new PropagationRequest);
	propRes_ = PropagationResultSharedPtr(new PropagationResult);
	obsReq_ = ObservationRequestSharedPtr(new ObservationRequest);

}

void ADVT::checkOptions_() {
	auto options = static_cast<const ADVTOptions *>(problemEnvironmentOptions_);
	if ((options->numDiameterSamples % 2) > 0)
		ERROR("'numDiameterSamples' must be an even value");
}


bool ADVT::reset() {
	auto options = static_cast<const ADVTOptions *>(problemEnvironmentOptions_);
	beliefTree_ = std::make_unique<Tree>();
	beliefTree_->initRoot<BeliefNode>();
	TreeElement *const root = beliefTree_->getRoot();

	// Pre-Sample particles
	LOGGING("Pre-sampling particles...");
	VectorRobotStatePtr particles(options->minParticleCount, nullptr);
	for (size_t i = 0; i != options->minParticleCount; ++i) {
		particles[i] = robotPlanningEnvironment_->sampleInitialState();

	}

	VectorFloat lowerActionBound;
	VectorFloat upperActionBound;
	robotPlanningEnvironment_->getRobot()->getActionSpace()->getActionLimits()->getLimits()->as<VectorLimitsContainer>()->get(lowerActionBound,
	        upperActionBound);

	initBeliefNode_(root);
	root->getData()->as<BeliefNodeData>()->setParticles(particles);
	return true;
}

bool ADVT::improvePolicy(const FloatType &timeout) {
	unsigned long numEpisodes = static_cast<const ADVTOptions *>(problemEnvironmentOptions_)->numEpisodes;
	numSampledEpisodes_ = 0;
	FloatType endTime = oppt::clock_ms() + timeout;
	while (true) {
		RobotStateSharedPtr state = beliefTree_->getRoot()->as<BeliefNode>()->sampleParticle();
		int depth = 0;
		search(beliefTree_->getRoot(), state, depth);
		numSampledEpisodes_++;
		if (numEpisodes > 0) {
			if (numSampledEpisodes_ == numEpisodes)
				break;
		} else {
			if (oppt::clock_ms() >= endTime)
				break;
		}
	}

	return true;
}

FloatType ADVT::search(TreeElement *currentBelief, RobotStateSharedPtr &state, int &depth) {
	auto options = static_cast<const ADVTOptions *>(problemEnvironmentOptions_);
	propRes_->nextState = state;
	if (robotPlanningEnvironment_->isTerminal(propRes_)) 		
		return 0.0;	

	bool requireHeuristic = false;
	if (currentBelief->as<BeliefNode>()->isNew()) {
		initBeliefNode_(currentBelief);
		requireHeuristic = true;
	}

	if (depth == options->maximumDepth) {
		requireHeuristic = true;
	}

	if (requireHeuristic) {
		std::shared_ptr<HeuristicInfo> heuristicInfo(new HeuristicInfo);
		heuristicInfo->currentState = state;
		heuristicInfo->discountFactor = problemEnvironmentOptions_->discountFactor;
		return heuristicPlugin_->getHeuristicValue(heuristicInfo.get());
	}

	depth++;
	TreeElement *currentBeliefNode = currentBelief;
	ObservationResultSharedPtr obsRes = nullptr;		

	// Select action
	auto actionInfo = currentBelief->as<BeliefNode>()->selectAction();
	auto action = actionInfo.first->as<PartitionNode>()->getAction();

	// Sample next state
	propReq_->currentState = state;
	propReq_->action = action;
	propRes_ = robotPlanningEnvironment_->getRobot()->propagateState(propReq_);

	// Sample observation
	obsReq_->currentState = propRes_->nextState;
	obsReq_->action = action;
	obsRes = robotPlanningEnvironment_->getRobot()->makeObservationReport(obsReq_);

	// Sample reward
	FloatType reward = robotPlanningEnvironment_->getReward(propRes_);

	// Go to the next belief
	state = propRes_->nextState;
	currentBelief = currentBelief->as<BeliefNode>()->getOrCreateChild<BeliefNode>(actionInfo.second,
	                obsRes->observation);

	FloatType expectedReward = reward + options->discountFactor * search(currentBelief, state, depth);	

	// Update statistics
	currentBeliefNode->as<BeliefNode>()->updateVisitCount(1);
	currentBeliefNode->as<BeliefNode>()->updateAgent(actionInfo.first, expectedReward);
	if (options->bellmanBackup and depth != 1)
		expectedReward = currentBeliefNode->as<BeliefNode>()->recalculateValue();
	return expectedReward;
}

void ADVT::stepFinished(const size_t &step) {

}

ActionSharedPtr ADVT::getNextAction() {
	cout << "GETTING ACTION FROM" << endl;
	beliefTree_->getRoot()->print();
	auto bestAct = beliefTree_->getRoot()->as<BeliefNode>()->getPartitionAgent()->getBestAction()->as<PartitionNode>()->getAction();
	auto actionEdge = beliefTree_->getRoot()->as<BeliefNode>()->getActionEdge(bestAct);
	if (actionEdge == nullptr)
		ERROR("Huh");

	cout << "NUM OBSERVATIONS: " << actionEdge->getNumChildren() << endl;
	//cout << "Tree depth: " << getSubtreeDepth(beliefTree_->getRoot(), 0) << endl;
	cout << "Num actions: " << beliefTree_->getRoot()->getNumChildren() << endl;
	cout << "Sampled " << numSampledEpisodes_ << " episodes" << endl;

	ActionSharedPtr bestAction(new VectorAction(bestAct->as<VectorAction>()->asVector()));
	//cout << "BEST ACTION: " << *(bestAction.get()) << endl;
	return bestAction;
}

bool ADVT::updateBelief(const ActionSharedPtr& action,
                        const ObservationSharedPtr& observation,
                        const bool &allowTerminalStates) {
	auto options = static_cast<const ADVTOptions *>(problemEnvironmentOptions_);

	VectorRobotStatePtr nextParticles;
	cout << "allowTerminalStates: " << allowTerminalStates << endl;
	if (options->rejectionSampling) {
		PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
		ObservationRequestSharedPtr observationRequest(new ObservationRequest);
		size_t numAttempts = 0;

		auto childBelief = beliefTree_->getRoot()->as<BeliefNode>()->getOrCreateChild<BeliefNode>(action.get(), observation);
		/**auto beliefData = childBelief->getData();
		if (beliefData) {
			nextParticles = childBelief->getData()->as<BeliefNodeData>()->getParticles();
		} else {
			LOGGING("Belief data is null");
		}*/
		LOGGING("Num next particles " + std::to_string(nextParticles.size()));
		while (nextParticles.size() < options->minParticleCount) {
			auto state = beliefTree_->getRoot()->as<BeliefNode>()->sampleParticle();
			propagationRequest->currentState = state;
			propagationRequest->action = action.get();
			propagationRequest->userData = OpptUserDataSharedPtr(new OpptUserData);
			auto propagationResult = robotPlanningEnvironment_->getRobot()->propagateState(propagationRequest);

			observationRequest->currentState = propagationResult->nextState;
			observationRequest->action = action.get();
			auto obsRes = robotPlanningEnvironment_->getRobot()->makeObservationReport(observationRequest);

			// Check if we're terminal
			if (allowTerminalStates) {
				nextParticles.push_back(propagationResult->nextState);
			} else {
				bool terminal = robotPlanningEnvironment_->isTerminal(propagationResult);
				if (terminal == false and observation->equals(*(obsRes->observation.get())))
					nextParticles.push_back(propagationResult->nextState);
			}


			numAttempts++;
			if (numAttempts > 100 * options->minParticleCount) {
				cout << "nextParticles.size(): " << nextParticles.size() << endl;
				if (nextParticles.size() < 0.5 * (FloatType)(options->minParticleCount)) {
					WARNING("Giving up");
					return false;
				} else {
					WARNING("Go on with " + std::to_string(nextParticles.size()) + " particles.");
				}
				break;
			}
		}

		/**if (nextParticles.size() < options->minParticleCount) {
			while (nextParticles.size() != options->minParticleCount) {
				propagationRequest->currentState = beliefTree_->getRoot()->as<BeliefNode>()->sampleParticle();
				propagationRequest->action = action.get();
				nextParticles.push_back(robotPlanningEnvironment_->getRobot()->propagateState(propagationRequest)->nextState);
			}
		}*/
	} else {
		auto randomEngine = robotPlanningEnvironment_->getRobot()->getRandomEngine();
		FilterRequestPtr filterRequest = std::make_unique<FilterRequest>();
		filterRequest->robotEnvironment = robotPlanningEnvironment_;
		filterRequest->allowZeroWeightParticles = false;
		filterRequest->numParticles = options->minParticleCount;
		filterRequest->action = action.get();
		filterRequest->observation = observation.get();
		filterRequest->allowCollisions = robotPlanningEnvironment_->getRobot()->getCollisionsAllowed();
		filterRequest->randomEngine = randomEngine;

		for (size_t i = 0; i != options->minParticleCount; ++i) {
			auto state = beliefTree_->getRoot()->as<BeliefNode>()->sampleParticle();
			filterRequest->previousParticles.push_back(std::make_shared<Particle>(state, 1.0));
		}


		FilterResultPtr filterResult = particleFilter_->filter(filterRequest);
		if (filterResult->particles.empty())
			return false;

		nextParticles = VectorRobotStatePtr(filterResult->particles.size(), nullptr);
		for (size_t i = 0; i != filterResult->particles.size(); ++i) {
			nextParticles[i] = filterResult->particles[i]->getState();
		}
	}

	TreeElement *const childBelief =
	    beliefTree_->getRoot()->as<BeliefNode>()->getOrCreateChild<BeliefNode>(action.get(),
	            observation);
	TreeElementPtr newBelief = std::move(childBelief->getParent()->releaseChild(childBelief));
	if (newBelief->as<BeliefNode>()->isNew() or options->resetTree)
		initBeliefNode_(newBelief.get());

	newBelief->getData()->as<BeliefNodeData>()->setParticles(nextParticles);
	beliefTree_->updateRoot(std::move(newBelief));
	cout << "numNextParticles: " << nextParticles.size() << endl;

	return true;
}

VectorRobotStatePtr ADVT::getBeliefParticles() {
	return beliefTree_->getRoot()->getData()->as<BeliefNodeData>()->getParticles();
}

void ADVT::initBeliefNode_(TreeElement * beliefNode) {
	beliefNode->as<BeliefNode>()->initialize(robotPlanningEnvironment_,
	        observationComparator_,
	        problemEnvironmentOptions_,
	        &lowerActionBound_,
	        &upperActionBound_,
	        rootDiameter_,
	        distanceMeasure_.get(),
	        diameterEstimator_.get());	
}

std::unique_ptr<Tree> ADVT::initPartitionTree_() {
	const ADVTOptions * options = static_cast<const ADVTOptions *>(problemEnvironmentOptions_);
	std::unique_ptr<Tree> partitionTree(new Tree);

	VectorFloat randomActionVec(lowerActionBound_.size(), 0.0);
	//VectorFloat randomAction(lowerActionBound_.size(), 0.0);
	auto randomEngine = robotPlanningEnvironment_->getRobot()->getRandomEngine().get();
	for (size_t i = 0; i != randomActionVec.size(); ++i) {
		std::uniform_real_distribution<FloatType> dist(lowerActionBound_[i], upperActionBound_[i]);
		randomActionVec[i] = dist(*randomEngine);
	}

	std::shared_ptr<oppt::Action> randomAction(new VectorAction(randomActionVec));

	TreeElementPtr initPartition = nullptr;
	if (options->partitioningMode == "RECTANGLE") {
		initPartition = TreeElementPtr(new RectanglePartition(nullptr,
		                               randomAction,
		                               lowerActionBound_,
		                               upperActionBound_,
		                               problemEnvironmentOptions_));
		//initPartition = PartitionPtr(new RectanglePartition(nullptr, lowerActionBound, upperActionBound));
	} else if (options->partitioningMode == "VORONOI") {
		initPartition = TreeElementPtr(new VoronoiPartition(randomAction,
		                               nullptr,
		                               &lowerActionBound_,
		                               &upperActionBound_,
		                               rootDiameter_,
		                               problemEnvironmentOptions_));
	} else {
		ERROR("Partitioning mode not recognized. Must be 'RECTANGLE' or 'VORONOI'");
	}

	partitionTree->updateRoot(std::move(initPartition));
	return std::move(partitionTree);
}

/**unsigned int ADVT::getSubtreeDepth(TreeElement *beliefNode, const unsigned int &nodeDepth) const {
	unsigned int nextNodeDepth = nodeDepth + 1;
	unsigned int maxSubtreeDepth = nodeDepth;

	// Iterate over the action edges of beliefNode
	auto actionEdge = beliefNode->getChildren();
	for (unsigned int i = 0; i != beliefNode->getNumChildren(); ++i) {

		// Iterate over the observation edges of the action edge
		auto observationEdge = (*actionEdge)->getChildren();
		for (unsigned int j = 0; j != (*actionEdge)->getNumChildren(); ++j) {

			// The next belief node associated to the observation edge
			auto nextBeliefNode = (*observationEdge)->getChildren();

			// Get the depth of the subtree under nextBeliefNode
			unsigned int subtreeDepth = getSubtreeDepth((*nextBeliefNode).get(), nextNodeDepth);
			if (subtreeDepth > maxSubtreeDepth)
				maxSubtreeDepth = subtreeDepth;

			observationEdge++;
		}

		actionEdge++;
	}

	return maxSubtreeDepth;
}*/



}