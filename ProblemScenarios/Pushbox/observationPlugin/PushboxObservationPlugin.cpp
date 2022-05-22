#ifndef _PUSHBOX_OBSERVATION_PLUGIN_HPP_
#define _PUSHBOX_OBSERVATION_PLUGIN_HPP_
#include <oppt/plugin/Plugin.hpp>
#include "PushboxObservationPluginOptions.hpp"
#include "TruncatedNormal.hpp"
#include "PushboxStateUserData.hpp"
#include "BearingObservation.hpp"
#include <oppt/opptCore/Distribution.hpp>

namespace oppt
{
class PushboxObservationPlugin: public ObservationPlugin
{
public :
    PushboxObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~PushboxObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxObservationPluginOptions>(optionsFile);
        auto options = static_cast<const PushboxObservationPluginOptions *>(options_.get());
        observationDistribution_ = std::make_unique<TruncatedNormalDistribution>(0.0, options->observationUncertainty);
        observationBucketFactor_ = 360.0 / ((FloatType)(options->numberOfObservationBuckets));
        if (options->usePositionObservation) {
            unsigned int observationSpaceDimension = robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions();
            Matrixdf mean = Matrixdf::Zero(observationSpaceDimension, 1);
            Matrixdf covarianceMatrix = Matrixdf::Identity(observationSpaceDimension, observationSpaceDimension);
            for (size_t i = 0; i != observationSpaceDimension; ++i) {
                covarianceMatrix(i, i) = options->positionObservationUncertainty;
            }

            auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
            positionDistribution_ =
                std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
            positionDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
            positionDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
        }
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        if (static_cast<const PushboxObservationPluginOptions *>(options_.get())->usePositionObservation)
            return getPositionObservation_(observationRequest);
        return getBearingObservation_(observationRequest);
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const override {
        if (static_cast<const PushboxObservationPluginOptions *>(options_.get())->usePositionObservation)
            return calcLikelihoodPositionObservation_(state, action, observation);
        return calcLikelihoodBearingObservation_(state, action, observation);
    }

private:
    std::unique_ptr<TruncatedNormalDistribution> observationDistribution_ = nullptr;

    std::unique_ptr<MultivariateNormalDistribution<FloatType>> positionDistribution_ = nullptr;

    const FloatType radians = 180.0 / M_PI;

    FloatType observationBucketFactor_ = 0.0;

private:
    ObservationResultSharedPtr getBearingObservation_(const ObservationRequest* observationRequest) const {
        ObservationResultSharedPtr observationResult(new ObservationResult);
        auto options = static_cast<const PushboxObservationPluginOptions *>(options_.get());
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        unsigned int numDims = stateVec.size() / 2;
        VectorFloat deltas(stateVec.size() / 2, 0.0);
        for (size_t i = 0; i != deltas.size(); ++i) {
            deltas[i] = stateVec[i + numDims] - stateVec[i];
        }

        VectorInt bearings(deltas.size() - 1, 0);        
        for (size_t i = 0; i != bearings.size(); ++i) {
            FloatType angle = std::atan2(deltas[i + 1], deltas[i]) * radians;
            angle += observationDistribution_->sample(*(robotEnvironment_->getRobot()->getRandomEngine().get()));
            if (angle < 0) angle += 360.0;
            if (angle > 360) angle -= 360.0;
            if (angle < 0) angle = 0.0;            
            FloatType bearing = std::floor(angle / observationBucketFactor_) * observationBucketFactor_;
            bearings[i] = bearing;
        }

        bool pushed = observationRequest->currentState->getUserData()->as<PushboxStateUserData>()->pushed;
        observationResult->observation = ObservationSharedPtr(new BearingObservation(bearings, pushed));
        return observationResult;
    }

    ObservationResultSharedPtr getPositionObservation_(const ObservationRequest* observationRequest) const {
        ObservationResultSharedPtr observationResult(new ObservationResult);
        auto options = static_cast<const PushboxObservationPluginOptions *>(options_.get());
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        unsigned int numDims = stateVec.size() / 2;
        VectorFloat observationVec(numDims, 0.0);
        for (size_t i = 0; i != numDims; ++i) {
            observationVec[i] = stateVec[numDims + i];
        }

        if (observationRequest->errorVector.empty() == false) {
            observationVec = addVectors(observationVec, observationRequest->errorVector);
        } else {
            observationVec = addVectors(observationVec, toStdVec<FloatType>(positionDistribution_->sample(1).col(0)));
        }

        bool pushed = observationRequest->currentState->getUserData()->as<PushboxStateUserData>()->pushed;
        observationResult->observation = ObservationSharedPtr(new PushboxPositionObservation(observationVec, pushed));
        return observationResult;
    }

    FloatType calcLikelihoodBearingObservation_(const RobotStateSharedPtr &state,
            const Action *action,
            const Observation *observation) const {
        /**auto obs = observation->as<BearingObservation>();
        if (state->getUserData()->as<PushboxStateUserData>()->pushed != obs->getPushed())
            return 0.0;

        VectorFloat stateVec = state->as<VectorState>()->asVector();
        unsigned int numDims = stateVec.size() / 2;
        auto obsBearings = obs->getBearings();

        VectorFloat deltas(stateVec.size() / 2, 0.0);
        for (size_t i = 0; i != deltas.size(); ++i) {
            deltas[i] = stateVec[i + numDims] - stateVec[i];
        }

        VectorInt bearings(deltas.size() - 1, 0);        
        for (size_t i = 0; i != bearings.size(); ++i) {
            FloatType angle = std::atan2(deltas[i + 1], deltas[i]) * radians;
            if (angle < 0) angle += 360.0;
            if (angle > 360) angle -= 360.0;
            if (angle < 0) angle = 0.0;            
            FloatType bearing = std::floor(angle / observationBucketFactor_) * observationBucketFactor_;
        }*/

        ObservationRequestSharedPtr observationRequest(new ObservationRequest());
        ObservationResultSharedPtr observationResult = nullptr;
        observationRequest->currentState = state;
        observationRequest->action = action;
        size_t numMatchingObservations = 0;
        for (size_t i = 0; i != 100; ++i) {
            observationResult = robotEnvironment_->getRobot()->makeObservationReport(observationRequest);
            if (observationResult->observation->equals(*observation))
                numMatchingObservations++;
        }

        if (numMatchingObservations == 0.0)
            return 0.0;
        return ((FloatType)(numMatchingObservations)) / 100.0;
    }

    FloatType calcLikelihoodPositionObservation_(const RobotStateSharedPtr &state,
            const Action *action,
            const Observation *observation) const {
        if (observation->as<PushboxPositionObservation>()->getPushed() != state->getUserData()->as<PushboxStateUserData>()->pushed) {
            return 0.0;
        }

        ObservationRequestSharedPtr observationRequest(new ObservationRequest());
        observationRequest->currentState = state;
        observationRequest->action = action;
        observationRequest->errorVector = VectorFloat(observationRequest->currentState->as<VectorState>()->asVector().size() / 2, 0.0);
        ObservationResultSharedPtr observationResult = robotEnvironment_->getRobot()->makeObservationReport(observationRequest);

        VectorFloat nominalObservationVec = observationResult->observation->as<DiscreteVectorObservation>()->asVector();
        VectorFloat observationVec = observation->as<DiscreteVectorObservation>()->asVector();
        VectorFloat diff = subtractVectors<FloatType>(observationVec, nominalObservationVec);
        if (std::accumulate(diff.begin(), diff.end(), 0.0) == 0.0) {
            return 1.0;
        }

        FloatType weight = positionDistribution_->pdf(diff);
        return weight;
    }
};

OPPT_REGISTER_OBSERVATION_PLUGIN(PushboxObservationPlugin)

#endif

}
