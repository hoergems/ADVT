#ifndef _ADVT_BELIEF_NODE_HPP_
#define _ADVT_BELIEF_NODE_HPP_
#include <OpptTree/TreeElement.hpp>
#include <oppt/opptCore/core.hpp>
#include "PartitionAgent.hpp"
#include "ActionEdge.hpp"
#include "ObservationEdge.hpp"

namespace oppt {
class BeliefNodeData;

class BeliefNode: public TreeElement {
public:
	friend class BeliefNodeData;
	BeliefNode(TreeElement *const parentElement);

	virtual ~BeliefNode() = default;

	bool isNew() const;

	void initialize(RobotEnvironment *robotEnvironment,
	                ObservationComparator observationComparator,
	                const ProblemEnvironmentOptions *options,
	                const VectorFloat *lowerActionBound,
	                const VectorFloat *upperActionBound,
	                const FloatType &rootDiameter,
	                const DistanceMeasure *distanceMeasure,
	                const DiameterEstimator *diameterEstimator);

	virtual void print() const override;

	RobotStateSharedPtr sampleParticle() const;

	TreeElement* getBestAction() const;

	TreeElement* getActionEdge(const Action *action) const;

	FloatType recalculateValue();

	FloatType getCachedValue();

	long getTotalVisitCount() const;

	void updateVisitCount(const long &visitCount);

	void updateAgent(TreeElement *node,
	                 const FloatType &reward);

	template<typename NodeType>
	TreeElement *const getOrCreateChild(const Action *action,
	                                    const ObservationSharedPtr &observation) {
		TreeElement *childActionEdge = nullptr;
		for (auto it = getChildren(); it != children_.end(); it++) {
			if ((*it)->as<ActionEdge>()->getAction()->equals(*action)) {
				childActionEdge = (*it).get();
				break;
			}
		}		

		TreeElement *const observationEdge = childActionEdge->as<ActionEdge>()->getOrCreateObservationEdge(observation);
		return observationEdge->as<ObservationEdge>()->getOrCreateChild<NodeType>();;
	}

	template<typename NodeType>
	TreeElement *const getOrCreateChild(const TreeElement *actionEdge,
	                                    const ObservationSharedPtr &observation) {
		TreeElement *childActionEdge = nullptr;
		for (auto it = getChildren(); it != children_.end(); it++) {
			if ((*it).get() == actionEdge) {
				childActionEdge = (*it).get();
				break;
			}
		}		

		TreeElement *const observationEdge = childActionEdge->as<ActionEdge>()->getOrCreateObservationEdge(observation);
		return observationEdge->as<ObservationEdge>()->getOrCreateChild<NodeType>();;
	}

	std::pair<TreeElement *, TreeElement *> const selectAction();

	PartitionAgent *getPartitionAgent() const;

protected:
	FloatType cachedValue_ = 0.0;

	long totalVisitCount_ = 0;

	std::unique_ptr<PartitionAgent> partitionAgent_ = nullptr;

	bool isNew_ = true;

	ObservationComparator observationComparator_;
};
}

#endif