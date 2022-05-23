#ifndef _PARTITION_AGENT_HPP_
#define _PARTITION_AGENT_HPP_
#include <oppt/opptCore/core.hpp>
#include "Tree.hpp"
#include "TreeElement.hpp"
#include "Partition.hpp"
#include "ADVTOptions.hpp"

namespace oppt {
class BeliefNode;
class PartitionAgent {
public:
	PartitionAgent(RobotEnvironment *robotEnvironment,
	               BeliefNode *associatedBeliefNode,
	               const ProblemEnvironmentOptions *options,
	               const VectorFloat *lowerActionBound,
	               const VectorFloat *upperActionBound,
	               const FloatType &rootDiameter,
	               const DistanceMeasure *distanceMeasure,
	               const DiameterEstimator *diameterEstimator);
	~PartitionAgent();

	_NO_COPY_BUT_MOVE(PartitionAgent)

	void print();

	/**
	 *@brief Return the leaf node with the largest Q-value
	 */
	TreeElement *getBestAction();

	/**
	 *@brief Return the leaf node with the largest Q-value + exploration terms
	 */
	TreeElement *selectAction();

	void updateReward(TreeElement *node,
	                  const FloatType &reward);

	std::vector<TreeElement *> getLeafNodes() const;

	void splitNode(TreeElement *node, const FloatType &explorationFactor, const FloatType &explorationFactorDiameter);

private:
	void initPartitionTree_(const VectorFloat *lowerActionBound, const VectorFloat *upperActionBound);

private:
	RobotEnvironment *robotEnvironment_ = nullptr;

	BeliefNode *associatedBeliefNode_ = nullptr;

	std::unique_ptr<Tree> partitionNodeTree_ = nullptr;

	Tree *partitionTree_ = nullptr;

	std::vector<ActionUniquePtr> allActions_;

	std::vector<TreeElement *> leafNodes_;

	std::unique_ptr<std::uniform_real_distribution<FloatType>> uniformDistribution_ = nullptr;

	FloatType rootDiameter_ = 0.0;

	const ProblemEnvironmentOptions *options_;

	const DistanceMeasure *distanceMeasure_;

	const DiameterEstimator *diameterEstimator_;

};
}

#endif