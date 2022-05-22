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
#ifndef _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#define _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
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

#endif
