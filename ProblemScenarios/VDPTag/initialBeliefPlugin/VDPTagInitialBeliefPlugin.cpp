#include <oppt/plugin/Plugin.hpp>
#include "VDPTagState.hpp"
#include "VDPTagUserData.hpp"

namespace oppt
{
class VDPTagInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    VDPTagInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~VDPTagInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        targetDistr_ = std::make_unique<std::uniform_real_distribution<FloatType>>(-4.0, 4.0);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        VectorFloat stateVec({0.0, 0.0, (*(targetDistr_.get()))(*randomEngine), (*(targetDistr_.get()))(*randomEngine)});
        RobotStateSharedPtr initState(new VDPTagState(stateVec));
        OpptUserDataSharedPtr ud(new VDPTagUserData);
        ud->as<VDPTagUserData>()->dist = (initState->as<VDPTagState>()->targetPos() - initState->as<VDPTagState>()->agentPos()).norm();
        initState->setUserData(ud);
        return initState;
    }

private:
    std::unique_ptr<std::uniform_real_distribution<FloatType>> targetDistr_ = nullptr;

};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(VDPTagInitialBeliefPlugin)

}