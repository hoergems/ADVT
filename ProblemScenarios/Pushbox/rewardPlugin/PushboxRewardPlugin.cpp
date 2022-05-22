#ifndef _PUSHBOX_REWARD_PLUGIN_HPP_
#define _PUSHBOX_REWARD_PLUGIN_HPP_
#include <oppt/plugin/Plugin.hpp>
#include "PushboxRewardOptions.hpp"
#include "PushboxStateUserData.hpp"

namespace oppt
{
class PushboxRewardPlugin: public RewardPlugin
{
public :
    PushboxRewardPlugin():
        RewardPlugin() {
    }

    virtual ~PushboxRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxRewardOptions>(optionsFile);
        auto options = static_cast<const PushboxRewardOptions *>(options_.get());
        moveCost_ = options->moveCost;
        goalReward_ = options->goalReward;
        collisionPenalty_ = options->collisionPenalty;
        return true;       
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        auto userData = propagationResult->nextState->getUserData()->as<PushboxStateUserData>();
        VectorFloat nextStateVec = propagationResult->nextState->as<VectorState>()->asVector();
        FloatType reward = -moveCost_;
        if (userData->isGoalState)
            reward += goalReward_;
        if (userData->isInCollision)
            reward -= collisionPenalty_;
        return reward;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::pair<FloatType, FloatType>(-moveCost_ - collisionPenalty_, -moveCost_ + goalReward_);
    }

private:
    FloatType moveCost_ = 0.0;
    FloatType goalReward_ = 0.0;
    FloatType collisionPenalty_ = 0.0;
};

OPPT_REGISTER_REWARD_PLUGIN(PushboxRewardPlugin)

}

#endif