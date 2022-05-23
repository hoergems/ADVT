/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#include <oppt/plugin/Plugin.hpp>
#include "SensorPlacementRewardOptions.hpp"
#include "SensorPlacementUserData.hpp"

namespace oppt
{
class SensorPlacementRewardPlugin: public RewardPlugin
{
public :
    SensorPlacementRewardPlugin():
        RewardPlugin() {

    }

    virtual ~SensorPlacementRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementRewardOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementRewardOptions *>(options_.get()); 
        stepPenalty_ = options->stepPenalty;
        illegalMovePenalty_ = options->illegalMovePenalty;
        exitReward_ = options->exitReward;       
        return true;       
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        auto ud = propagationResult->nextState->getUserData()->as<SensorPlacementUserData>();
        FloatType reward = -stepPenalty_;
        if (ud->collides) {
            reward -= illegalMovePenalty_;
        } else if (ud->reachedGoal) {        
            reward += exitReward_;        
        }

        return reward;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::pair<FloatType, FloatType>(-stepPenalty_-illegalMovePenalty_, -stepPenalty_ + exitReward_);
    }

private:
    FloatType stepPenalty_ = 0.0;

    FloatType illegalMovePenalty_ = 0.0;

    FloatType exitReward_ = 0.0;
};

OPPT_REGISTER_REWARD_PLUGIN(SensorPlacementRewardPlugin)

}
