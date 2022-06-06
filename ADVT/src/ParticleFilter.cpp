#include "ParticleFilter.hpp"

namespace oppt {
ADVTFilter::ADVTFilter():
	ParticleFilter() {

}

FilterResultPtr ADVTFilter::propagateParticles(const FilterRequestPtr& filterRequest) {
	long currNumNextParticles = filterRequest->currentNextParticles.size();
	int deficit = filterRequest->numParticles - currNumNextParticles;
	if (deficit <= 0) {
		FilterResultPtr filterResult = std::make_unique<FilterResult>();
		filterResult->particles = filterRequest->currentNextParticles;
		return filterResult;
	}

	ParticleSet particleSet(filterRequest.get());
	VectorParticles replenishedParticlesVec(deficit, nullptr);

	unsigned int particleCounter = 0;
	PropagationRequestSharedPtr propagationRequest;
	auto robot = filterRequest->robotEnvironment->getRobot();
	if (deficit > 0) {
		auto sampledParticles = particleSet.sampleWeighted(filterRequest->randomEngine, deficit);
		for (size_t i = particleCounter; i != deficit; ++i) {
			propagationRequest = std::make_shared<PropagationRequest>();
			propagationRequest->currentState = sampledParticles[i]->getState();
			propagationRequest->action = filterRequest->action;
			propagationRequest->allowCollisions = filterRequest->allowCollisions;
			PropagationResultSharedPtr propagationRes = robot->propagateState(propagationRequest);

			if (filterRequest->robotEnvironment->isTerminal(propagationRes) == false or filterRequest->allowTerminalStates) {
				auto propagatedParticle = std::make_shared<Particle>(propagationRes->nextState, sampledParticles[i]->getWeight());
				// Update the resulting particle weight according to the observation model p(y | x)
				FloatType newWeight = sampledParticles[i]->getWeight();
				if (propagatedParticle->getWeight() > 0 || filterRequest->allowZeroWeightParticles) {
					replenishedParticlesVec[particleCounter] = propagatedParticle;
					OpptUserDataSharedPtr userData = replenishedParticlesVec[particleCounter]->getState()->getUserData();
					if (userData) {
						userData->as<RobotStateUserData>()->previousState = sampledParticles[i]->getState();
					} else {
						userData = std::make_shared<RobotStateUserData>();
						userData->as<RobotStateUserData>()->previousState = sampledParticles[i]->getState();
						replenishedParticlesVec[particleCounter]->getState()->setUserData(userData);
					}

					particleCounter++;
				}
			}
		}
	}

	FilterResultPtr filterResult = std::make_unique<FilterResult>();
    filterResult->particles.resize(currNumNextParticles + particleCounter);
    for (size_t i = 0; i != currNumNextParticles; ++i) {
        filterResult->particles[i] = filterRequest->currentNextParticles[i];
    }
    for (size_t i = 0; i != particleCounter; ++i) {
        filterResult->particles[currNumNextParticles + i] = replenishedParticlesVec[i];
    }

    return std::move(filterResult);
}
}