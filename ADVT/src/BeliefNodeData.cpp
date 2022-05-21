#include "BeliefNodeData.hpp"
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>


namespace oppt {
BeliefNodeData::BeliefNodeData(TreeElement *const parentElement, RobotEnvironment *robotEnvironment):
	TreeElementData(parentElement),
	robotEnvironment_(robotEnvironment) {

}

void BeliefNodeData::setParticles(VectorRobotStatePtr &particles) {
	particles_ = particles;
	unformParticleDistribution_ =
	    std::make_unique<std::uniform_int_distribution<unsigned int>>(0, particles.size() - 1);
}

void BeliefNodeData::addParticle(RobotStateSharedPtr &state) {
	particles_.push_back(state);
}

VectorRobotStatePtr BeliefNodeData::getParticles() const {
	return particles_;
}

size_t BeliefNodeData::getNumParticles() const {
	return particles_.size();
}

RobotStateSharedPtr BeliefNodeData::sampleParticle() const {
	if (particles_.empty())
		return robotEnvironment_->sampleInitialState();
	return particles_[(*(unformParticleDistribution_.get()))(*(robotEnvironment_->getRobot()->getRandomEngine().get()))];
}
}