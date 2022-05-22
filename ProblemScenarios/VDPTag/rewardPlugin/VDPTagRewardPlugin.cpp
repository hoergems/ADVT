#include <oppt/plugin/Plugin.hpp>
#include "VDPTagState.hpp"
#include "VDPTagUserData.hpp"

namespace oppt
{
class VDPTagRewardPlugin: public RewardPlugin
{
public :
    VDPTagRewardPlugin():
        RewardPlugin() {
    }

    virtual ~VDPTagRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        FloatType dist =
            (propagationResult->nextState->as<VDPTagState>()->targetPos() - propagationResult->nextState->as<VDPTagState>()->agentPos()).norm();

        FloatType reward = 0.0;
        if (dist < tagRadius_) {
            reward = tagReward_;
        } else {
            reward = -stepCost_;
        }
        
        if (propagationResult->action->as<VectorAction>()->asVector()[1] > 0.5)
            reward -= lookCost_;

        return reward;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::make_pair(-6.0,
                              100.0);
    }

private:
    FloatType tagRadius_ = 0.1;

    FloatType tagReward_ = 100.0;

    FloatType stepCost_ = 1.0;

    FloatType lookCost_ = 5.0;
};

OPPT_REGISTER_REWARD_PLUGIN(VDPTagRewardPlugin)

}
