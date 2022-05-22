#include <oppt/plugin/Plugin.hpp>
#include "ParkingInitialBeliefOptions.hpp"
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "VehicleState.hpp"
#include "VehicleUserData.hpp"

namespace oppt
{
class ParkingInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    ParkingInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~ParkingInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ParkingInitialBeliefOptions>(optionsFile);
        auto options = static_cast<const ParkingInitialBeliefOptions *>(options_.get());
        numStateDims_ = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions();
        distr_ = std::make_unique<std::uniform_real_distribution<FloatType>>(0.0, 1.0);
        positionErrorDistr_ = std::make_unique<std::uniform_real_distribution<FloatType>>(-options->positionError, options->positionError);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        VectorFloat initStateVec(5, 0.0);
        FloatType positionError = (*(positionErrorDistr_.get()))(*randomEngine);
        initStateVec[0] = -1.0;
        initStateVec[1] = positionError;
        initStateVec[3] = 0.1;
        initStateVec[3] = 0.0;
        FloatType randomNumber = (*(distr_.get()))(*randomEngine);
        if (randomNumber <= (1.0 / 3.0)) {
            initStateVec[1] = -3.0 + positionError;
        } else if (randomNumber > (2.0 / 3.0)) {
            initStateVec[1] = 3.0 + positionError;
        }

        auto world = robotEnvironment_->getGazeboInterface()->getWorld();
        world->Reset();
        world->ResetPhysicsStates();
        world->ResetTime();

        // First construct an intial world state based on the intial state vector
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initStateVec, false);
        RobotStateSharedPtr initState(new VehicleState(initStateVec));
        RobotStateUserDataSharedPtr userData(new VehicleUserData);
        initState->setUserData(userData);
        return initState;
    }

private:
    unsigned int numStateDims_ = 0;

    std::unique_ptr<std::uniform_real_distribution<FloatType>> distr_ = nullptr;
    std::unique_ptr<std::uniform_real_distribution<FloatType>> positionErrorDistr_ = nullptr;
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(ParkingInitialBeliefPlugin)

}