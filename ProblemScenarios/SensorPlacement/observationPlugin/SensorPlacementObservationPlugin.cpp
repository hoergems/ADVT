#include <oppt/plugin/Plugin.hpp>
#include "SensorPlacementObservationPluginOptions.hpp"
#include "SensorPlacementUserData.hpp"

namespace oppt
{
class SensorPlacementObservationPlugin: public ObservationPlugin
{
public :
    SensorPlacementObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~SensorPlacementObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementObservationPluginOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementObservationPluginOptions *>(options_.get());
        correctObservationProbability_ = options->correctObservationProbability;
        distr_ = std::make_unique<std::uniform_real_distribution<FloatType>>(0.0, 1.0);
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        ObservationResultSharedPtr observationResult(new ObservationResult);
        bool correctObservation = (*(distr_.get()))(*(robotEnvironment_->getRobot()->getRandomEngine().get())) < correctObservationProbability_ ? true : false;
        FloatType observationVal = 0.0;
        auto ud = observationRequest->currentState->getUserData()->as<SensorPlacementUserData>();
        if (ud->eeTouchesWall and correctObservation) {            
            if (ud->eePose.position.y() < 0.0) {
                if (ud->eePose.position.z() > 1.5) {
                    observationVal = 1.0;
                } else {
                    observationVal = 2.0;
                }
            } else {
                if (ud->eePose.position.z() > 1.5) {
                    observationVal = 3.0;
                } else {
                    observationVal = 4.0;
                }
            }            
        }

        observationResult->observation = ObservationSharedPtr(new VectorObservation({observationVal}));

        return observationResult;

    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const override {
        bool stateEETouchesWall = state->getUserData()->as<SensorPlacementUserData>()->eeTouchesWall;
        FloatType observationVal = observation->as<VectorObservation>()->asVector()[0];
        FloatType weight = 0.0;        
        if (stateEETouchesWall) {
            FloatType y = state->getUserData()->as<SensorPlacementUserData>()->eePose.position.y();
            FloatType z = state->getUserData()->as<SensorPlacementUserData>()->eePose.position.z();
            if (y < 0.0) {
                if (z > 1.5) {
                    weight = std::fabs(observationVal - 1.0) < 0.5 ? correctObservationProbability_ : 1.0 - correctObservationProbability_;
                } else {
                    weight = std::fabs(observationVal - 2.0) < 0.5 ? correctObservationProbability_ : 1.0 - correctObservationProbability_;
                }

                // should get left observation
                /**if (std::fabs(observationVal - 1.0) < 0.5) {
                    weight = 1.0;
                } else {
                    weight = 0.0;
                }*/
            } else {
                if (z > 1.5) {
                    weight = std::fabs(observationVal - 3.0) < 0.5 ? correctObservationProbability_ : 1.0 - correctObservationProbability_;
                } else {
                    weight = std::fabs(observationVal - 4.0) < 0.5 ? correctObservationProbability_ : 1.0 - correctObservationProbability_;
                }
                // should get right observation
                /**if (std::fabs(observationVal - 2.0) < 0.5) {
                    weight = 1.0;
                } else {
                    weight = 0.0;
                }*/
            }            
        } else {
            if (observationVal < 0.5) {
                weight = 1.0;
            } else {
                weight = 0.01;
            }
        }


        /**if (stateEETouchesWall) {
            if (observationVal > 0.5) {
                weight =  correctObservationProbability_;
            } else {
                weight = 1.0 - correctObservationProbability_;
            }
        } else {
            if (observationVal > 0.5) {
                weight = 1.0 - correctObservationProbability_;
            } else {
                weight = correctObservationProbability_;
            }
        }*/

        return weight;
    }

private:
    std::unique_ptr<std::uniform_real_distribution<FloatType>> distr_ = nullptr;

    FloatType correctObservationProbability_ = 0.0;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(SensorPlacementObservationPlugin)

}
