#include <oppt/plugin/Plugin.hpp>
#include "PushboxInitialBeliefOptions.hpp"
#include "TruncatedNormal.hpp"
#include "PushboxStateUserData.hpp"

namespace oppt
{
class PushboxInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    PushboxInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~PushboxInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxInitialBeliefOptions>(optionsFile);
        auto options = static_cast<const PushboxInitialBeliefOptions *>(options_.get());
        truncatedNormal_ = std::make_unique<TruncatedNormalDistribution>(0.0, options->initialBoxPositionUncertainty);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        VectorFloat initialStateVec = static_cast<const PushboxInitialBeliefOptions *>(options_.get())->initialStateVec;
        unsigned int stateHalfDim = initialStateVec.size() / 2;
        for (size_t i = 0; i != stateHalfDim; ++i) {
            initialStateVec[i + stateHalfDim] += truncatedNormal_->sample((*(robotEnvironment_->getRobot()->getRandomEngine().get())));
        }

        RobotStateSharedPtr initialState(new VectorState(initialStateVec));
        OpptUserDataSharedPtr userData(new PushboxStateUserData);
        userData->as<PushboxStateUserData>()->isGoalState = false;
        userData->as<PushboxStateUserData>()->isInCollision = false;
        initialState->setUserData(userData);
        return initialState;
    }

private:
    std::unique_ptr<TruncatedNormalDistribution> truncatedNormal_ = nullptr;
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(PushboxInitialBeliefPlugin)

}