#include <oppt/plugin/Plugin.hpp>
#include "PushboxStateUserData.hpp"
#include "PushboxHeuristicOptions.hpp"

namespace oppt
{
class PushboxHeuristicPlugin: public HeuristicPlugin
{
public:
    PushboxHeuristicPlugin():
        HeuristicPlugin() {
    }

    virtual ~PushboxHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxHeuristicOptions>(optionsFile);
        auto options = static_cast<const PushboxHeuristicOptions *>(options_.get());
        moveCost_ = options->moveCost;
        goalReward_ = options->goalReward;
        collisionPenalty_ = options->collisionPenalty;
        goalPosition_ = options->goalPosition;

        // Extend the goal position dimension
        unsigned int numStateDimensionsHalf = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions() / 2;
        if (numStateDimensionsHalf > 2) {
            VectorFloat numAdditionalGoalPositionDimensions(numStateDimensionsHalf - 2, 0.0);
            goalPosition_.insert(goalPosition_.end(), numAdditionalGoalPositionDimensions.begin(), numAdditionalGoalPositionDimensions.end());
        }
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        auto userData = heuristicInfo->currentState->getUserData()->as<PushboxStateUserData>();
        if (userData->isGoalState)
            return goalReward_;
        if (userData->isInCollision)
            return -collisionPenalty_;

        FloatType heuristicValue = getUpperBoundHeuristicValue(heuristicInfo->currentState->as<VectorState>()->asVector());
        return heuristicValue;
    }

private:
    FloatType moveCost_ = 0.0;
    FloatType goalReward_ = 0.0;
    FloatType collisionPenalty_ = 0.0;

    VectorFloat goalPosition_;
private:
    FloatType getUpperBoundHeuristicValue(const VectorFloat &stateVec) const {
        auto options = static_cast<const PushboxHeuristicOptions *>(options_.get());
        FloatType result = 0.0;

        unsigned int stateHalfDim = stateVec.size() / 2;
        VectorFloat robotPosition(stateVec.begin(), stateVec.begin() + stateHalfDim);
        VectorFloat opponentPosition(stateVec.begin() + stateHalfDim, stateVec.end());
        FloatType distanceToGoal = math::euclideanDistance(opponentPosition, goalPosition_);
        VectorFloat goalToBox = subtractVectors(opponentPosition, goalPosition_);

        VectorFloat attractionPoint = addVectors(opponentPosition, scaleVector(goalToBox, 1.0 / distanceToGoal));
        distanceToGoal += math::euclideanDistance(robotPosition, attractionPoint);

        FloatType discountPower = std::pow(options->discountFactor , distanceToGoal);
        result -= moveCost_ * (discountPower - 1) / std::log(options->discountFactor);
        result += goalReward_ * discountPower;
        return result;
    }
};

OPPT_REGISTER_HEURISTIC_PLUGIN(PushboxHeuristicPlugin)

}