#ifndef _ADVT_HPP_
#define _ADVT_HPP_
#include <oppt/solver/solver.hpp>
//#include <OpptTree/TreeElement.hpp>
#include <oppt/filter/particleFilter/ParticleFilter.hpp>
#include "Tree.hpp"
#include "ObservationComparator.hpp"
#include "DistanceMeasure.hpp"
#include "DiameterEstimator.hpp"

using namespace oppt;

namespace solvers {
class ADVT: public Solver {
public:
	ADVT();

	~ADVT() = default;

	void setup() override;

	bool reset() override;

	bool improvePolicy(const FloatType &timeout) override;

	ActionSharedPtr getNextAction() override;

	bool updateBelief(const ActionSharedPtr& action,
	                  const ObservationSharedPtr& observation,
	                  const bool &allowTerminalStates = false) override;

	VectorRobotStatePtr getBeliefParticles() override;

	virtual void stepFinished(const size_t &step) override;

private:
	void initBeliefNode_(TreeElement *beliefNode);

	void checkOptions_();

	//unsigned int getSubtreeDepth(TreeElement *beliefNode, const unsigned int &nodeDepth) const;

	std::unique_ptr<Tree> initPartitionTree_();		

	FloatType search(TreeElement *currentBelief, RobotStateSharedPtr &state, int &depth);

	bool improvePolicy2(const FloatType &timeout);

private:
	std::unique_ptr<Tree> beliefTree_ = nullptr;

	VectorFloat lowerActionBound_;
	VectorFloat upperActionBound_;

	FloatType rootDiameter_ = 0.0;

	ObservationComparator observationComparator_;

	std::unique_ptr<ParticleFilter> particleFilter_ = nullptr;

	std::unique_ptr<DistanceMeasure> distanceMeasure_ = nullptr;

	std::unique_ptr<DiameterEstimator> diameterEstimator_ = nullptr;

	size_t numSampledEpisodes_ = 0;


	////////////////////////
	PropagationRequestSharedPtr propReq_;

	PropagationResultSharedPtr propRes_;

	ObservationRequestSharedPtr obsReq_;
};

}

#endif