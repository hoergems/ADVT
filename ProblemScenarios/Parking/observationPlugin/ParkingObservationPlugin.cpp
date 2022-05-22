#include <oppt/plugin/Plugin.hpp>
#include "ParkingObservationPluginOptions.hpp"
#include "VehicleUserData.hpp"

namespace oppt
{
class ParkingObservationPlugin: public ObservationPlugin
{
public :
    using ObservationPlugin::ObservationPlugin;
    _NO_COPY_BUT_MOVE(ParkingObservationPlugin)

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ParkingObservationPluginOptions>(optionsFile);
        correctObservationProbability_ = static_cast<const ParkingObservationPluginOptions *>(options_.get())->correctObservationProbability;
        uniformDistr_ = std::make_unique<std::uniform_real_distribution<FloatType>>(0.0, 1.0);
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        ObservationResultSharedPtr observationResult(new ObservationResult);
        int tile = observationRequest->currentState->getUserData()->as<VehicleUserData>()->aboveTile;
        if (tile == 0) {
            observationResult->observation = ObservationSharedPtr(new DiscreteVectorObservation({tile}));
        } else {
            // With some probability, we observe the wrong tile
            if ((*(uniformDistr_.get()))(*randomEngine) <= correctObservationProbability_) {
                // CorrectObservation
                observationResult->observation = ObservationSharedPtr(new DiscreteVectorObservation({tile}));
            } else {
                // Wrong observation
                int observedTile = 0;                
                if (tile == 1) {
                    observedTile = (*(uniformDistr_.get()))(*randomEngine) < 0.5 ? 2 : 3;
                } else if (tile == 2) {
                    observedTile = (*(uniformDistr_.get()))(*randomEngine) < 0.5 ? 1 : 3;
                } else if (tile == 3) {
                    observedTile = (*(uniformDistr_.get()))(*randomEngine) < 0.5 ? 1 : 2;
                }

                observationResult->observation = ObservationSharedPtr(new DiscreteVectorObservation({observedTile}));
            }
        }
        observationResult->observation = ObservationSharedPtr(new DiscreteVectorObservation({tile}));
        observationResult->observation->as<DiscreteVectorObservation>()->setBinNumber(tile);
        return observationResult;
    }

    virtual double calcLikelihood(const RobotStateSharedPtr &state,
                                  const Action *action,
                                  const Observation *observation) const override {
        int aboveTileState = state->getUserData()->as<VehicleUserData>()->aboveTile;
        int aboveTileObservation = observation->as<DiscreteVectorObservation>()->getBinNumber();

        if (aboveTileObservation == 0) {
            if (aboveTileState != 0)
                return 0.0;
            return 1.0;
        } else {
            if (aboveTileState == aboveTileObservation)
                return correctObservationProbability_;
            return 1.0 - correctObservationProbability_;
        }


        ERROR("We should not be here");
    }

private:
    FloatType correctObservationProbability_ = 0.0;

    std::unique_ptr<std::uniform_real_distribution<FloatType>> uniformDistr_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(ParkingObservationPlugin)

}
