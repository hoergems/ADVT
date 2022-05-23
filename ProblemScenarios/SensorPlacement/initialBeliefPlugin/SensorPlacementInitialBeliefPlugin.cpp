#include <oppt/plugin/Plugin.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "SensorPlacementInitialBeliefOptions.hpp"
#include "SensorPlacementUserData.hpp"

namespace oppt
{
class SensorPlacementInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    SensorPlacementInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~SensorPlacementInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementInitialBeliefOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementInitialBeliefOptions *>(options_.get());
        auto re = robotEnvironment_->getRobot()->getRandomEngine();
        VectorFloat lowerBound = options->lowerUpperBound;
        VectorFloat upperBound = options->lowerUpperBound;
        for (size_t i = 0; i != lowerBound.size(); ++i) {
            lowerBound[i] *= -1.0;
        }

        VectorFloat l({0.0});
        VectorFloat u({1.0});

        distr_ = std::make_unique<UniformDistribution<FloatType>>(lowerBound, upperBound, re);
        distr2_ = std::make_unique<UniformDistribution<FloatType>>(l, u, re);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        VectorFloat initialStateVec = static_cast<const SensorPlacementInitialBeliefOptions *>(options_.get())->initialStateVec;
        FloatType firstAngle = initialStateVec[0];
        initialStateVec = addVectors(initialStateVec, toStdVec<FloatType>(distr_->sample(1).col(0)));

        /**if (distr2_->sample(1)(0, 0) <= 0.5) {
            initialStateVec[0] = firstAngle-0.1;
        } else {
            initialStateVec[0] = firstAngle+0.1;
        }*/

        //VectorFloat error = toStdVec(distr_->sample(1).col(0));

        // First construct an intial world state based in the intial state vector
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initialStateVec);

        // Then get the initial world state
        GazeboWorldStatePtr initialWorldState = robotEnvironment_->getGazeboInterface()->getInitialWorldState();

        RobotStateSharedPtr initialState(new VectorState(initialStateVec));
        initialState->setGazeboWorldState(initialWorldState);

        OpptUserDataSharedPtr userData(new SensorPlacementUserData);
        userData->as<SensorPlacementUserData>()->collides = false;
        userData->as<SensorPlacementUserData>()->eeTouchesWall = false;
        userData->as<SensorPlacementUserData>()->reachedGoal = false;
        initialState->setUserData(userData);
        return initialState;
    }

private:
    std::unique_ptr<UniformDistribution<FloatType>> distr_ = nullptr;

    std::unique_ptr<UniformDistribution<FloatType>> distr2_ = nullptr;
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(SensorPlacementInitialBeliefPlugin)

}