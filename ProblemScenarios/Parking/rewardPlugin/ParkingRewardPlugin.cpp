#include <oppt/plugin/Plugin.hpp>
#include "ParkingRewardOptions.hpp"
#include "VehicleUserData.hpp"

namespace oppt
{
class ParkingRewardPlugin: public RewardPlugin
{
public :
    ParkingRewardPlugin():
        RewardPlugin() {

    }

    virtual ~ParkingRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ParkingRewardOptions>(optionsFile);
        stepPenalty = static_cast<const ParkingRewardOptions *>(options_.get())->stepPenalty;
        exitReward = static_cast<const ParkingRewardOptions *>(options_.get())->exitReward;
        collisionPenalty = static_cast<const ParkingRewardOptions *>(options_.get())->collisionPenalty;         
        
        return true;
    }

    virtual double getReward(const PropagationResultSharedPtr& propagationResult) const override {
        auto ud = propagationResult->nextState->getUserData()->as<VehicleUserData>();
        FloatType reward = -stepPenalty;
        if (ud->insideGoalArea)
            reward += exitReward;
        if (ud->collides)
            reward -= collisionPenalty;
        return reward;                   
        
    }

    virtual std::pair<double, double> getMinMaxReward() const override {
        return std::make_pair(-stepPenalty,
                              -stepPenalty + exitReward);
    }

private:
    FloatType stepPenalty = 0.0;

    FloatType exitReward = 0.0;

    FloatType collisionPenalty = 0.0;    

};

OPPT_REGISTER_REWARD_PLUGIN(ParkingRewardPlugin)

}
