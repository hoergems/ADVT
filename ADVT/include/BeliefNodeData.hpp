#ifndef _BELIEF_NODE_DATA_HPP_
#define _BELIEF_NODE_DATA_HPP_
#include <oppt/opptCore/core.hpp>
#include "TreeElementData.hpp"

namespace oppt {
class BeliefNodeData: public TreeElementData {
public:
	BeliefNodeData(TreeElement *const parentElement, RobotEnvironment *robotEnvironment);
	virtual ~BeliefNodeData() = default;
	_NO_COPY_BUT_MOVE(BeliefNodeData)

	void setParticles(VectorRobotStatePtr &particles);

	void addParticle(RobotStateSharedPtr &state);

	VectorRobotStatePtr getParticles() const;

	size_t getNumParticles() const;

	RobotStateSharedPtr sampleParticle() const;

private:
	VectorRobotStatePtr particles_;

	RobotEnvironment *robotEnvironment_ = nullptr;

	std::unique_ptr<std::uniform_int_distribution<unsigned int>> unformParticleDistribution_;

};
}

#endif