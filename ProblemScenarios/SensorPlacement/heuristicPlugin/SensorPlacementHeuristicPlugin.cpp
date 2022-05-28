#include <oppt/plugin/Plugin.hpp>
#include "SensorPlacementHeuristicOptions.hpp"
#include "SensorPlacementUserData.hpp"
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "GoalParser.hpp"

namespace oppt
{
class SensorPlacementHeuristicPlugin: public HeuristicPlugin
{
public:
    SensorPlacementHeuristicPlugin():
        HeuristicPlugin() {

    }

    virtual ~SensorPlacementHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementHeuristicOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementHeuristicOptions *>(options_.get());
        eeLink_ = getLinkPtr(robotEnvironment_, options->endEffectorLink);
        if (eeLink_ == nullptr)
            ERROR("EE link link couldn't be found");
        parseGoal_();

        exitReward_ = options->exitReward;
        stepPenalty_ = options->stepPenalty;
        discountFactor_ = options->discountFactor;

        VectorFloat lowerActionLimits;
        VectorFloat upperActionLimits;
        auto cont = robotEnvironment_->getRobot()->getActionSpace()->getActionLimits()->getLimits()->as<VectorLimitsContainer>();
        cont->get(lowerActionLimits, upperActionLimits);

        for (size_t i = 0; i != lowerActionLimits.size(); ++i) {
            distrs_.push_back(std::unique_ptr<std::uniform_real_distribution<FloatType>>(new
                              std::uniform_real_distribution<FloatType>(lowerActionLimits[i], upperActionLimits[i])));
        }
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        //return rollout(heuristicInfo->currentState);
        auto userData = heuristicInfo->currentState->getUserData()->as<SensorPlacementUserData>();
        VectorFloat eePosition({eeLink_->WorldPose().Pos().X(), eeLink_->WorldPose().Pos().Y(), eeLink_->WorldPose().Pos().Z()});
        //VectorFloat eePosition({eeLink_->WorldPose().Pos().X() - 0.05, eeLink_->WorldPose().Pos().Y(), eeLink_->WorldPose().Pos().Z()});
        FloatType distToGoal = math::euclideanDistance(eePosition, goalPosition_);
        return getHeuristicValueTest(distToGoal, userData->eeTouchesWall);
        //return getHeuristicValueOrig(distToGoal, userData->eeTouchesWall);
    }

private:
    FloatType discountFactor_ = 0.0;

    FloatType stepPenalty_ = 0.0;

    FloatType exitReward_ = 0.0;

    gazebo::physics::Link *eeLink_ = nullptr;

    VectorFloat goalPosition_;

    FloatType goalRadius_ = 0.0;

    FloatType alpha = 0.9;

    unsigned int numIterationsRollout = 5;

    std::vector<std::unique_ptr<std::uniform_real_distribution<FloatType>>> distrs_;

private:
    ActionSharedPtr randomAction() const {
        VectorFloat randomAction(distrs_.size(), 0.0);
        for (size_t i = 0; i != distrs_.size(); ++i) {
            randomAction[i] = (*(distrs_[i].get()))(*(robotEnvironment_->getRobot()->getRandomEngine().get()));
        }

        ActionSharedPtr action(new VectorAction(randomAction));
        return action;
    }

    FloatType rollout(const RobotStateSharedPtr &state) const {
        RobotStateSharedPtr currentState(new VectorState(state->as<VectorState>()->asVector()));
        ActionSharedPtr action = nullptr;
        PropagationRequestSharedPtr propReq(new PropagationRequest);
        PropagationResultSharedPtr propRes;
        FloatType reward = 0.0;
        FloatType currentDiscount = 1.0;
        FloatType totalDiscountedReward = 0.0;
        for (int i = 0; i != numIterationsRollout; ++i) {
            action = randomAction();

            propReq->currentState = currentState;
            propReq->action = action.get();

            propRes = robotEnvironment_->getRobot()->propagateState(propReq);
            reward = robotEnvironment_->getReward(propRes);
            currentState = propRes->nextState;

            totalDiscountedReward += currentDiscount * reward;
            currentDiscount *= discountFactor_;

            if (robotEnvironment_->isTerminal(propRes)) {
                break;
            }
        }

        return totalDiscountedReward;
    }


    // USED IN EXPERIMENTS
    FloatType getHeuristicValueOrig(const FloatType &distToGoal, const bool &eeTouchesWall) const {
        FloatType heuristic = 0.0;
        FloatType discountPower = std::pow(discountFactor_, distToGoal);
        heuristic -= stepPenalty_ * (discountPower - 1.0) / std::log(discountFactor_);
        heuristic += discountPower * exitReward_;
        //FloatType heuristic = exitReward_ * exp(-distToGoal);
        if (eeTouchesWall) {
            heuristic = alpha * exitReward_ + (1.0 - alpha) * heuristic;
        }

        return heuristic;
    }

    FloatType getHeuristicValueTest(const FloatType &distToGoal, const bool &eeTouchesWall) const {
        //return 0.0;
        FloatType heuristic = 0.0;
        //FloatType df = 0.1;
        FloatType df = 0.001;
        //FloatType discountPower = std::pow(discountFactor_, distToGoal);
        FloatType discountPower = std::pow(df, distToGoal);
        heuristic -= stepPenalty_ * (discountPower - 1.0) / std::log(df);
        heuristic += discountPower * exitReward_;
        //FloatType heuristic = exitReward_ * exp(-distToGoal);
        /**if (eeTouchesWall) {
            heuristic = alpha * exitReward_ + (1.0-alpha) * heuristic;
        }*/

        return heuristic;
    }



    gazebo::physics::Link *getLinkPtr(const RobotEnvironment *robotEnvironment, const std::string &name) {
        auto links = robotEnvironment->getGazeboInterface()->getLinks();
        gazebo::physics::Link *foundLink = nullptr;
        for (auto &link : links) {
            if (link->GetScopedName() == name) {
                foundLink = link;
                break;
            }
        }

        if (!foundLink) {
            WARNING("Link '" + name + "' could not be found");
            getchar();
        }

        return foundLink;
    }

    void parseGoal_() {
        GoalParser p;
        VectorFloat goalArea = p.parseGoalAreaFromFile(robotEnvironment_->getWorldFile());
        goalPosition_ = VectorFloat({goalArea[0], goalArea[1], goalArea[2]});
        goalRadius_ = goalArea[3];
    }

};

OPPT_REGISTER_HEURISTIC_PLUGIN(SensorPlacementHeuristicPlugin)

}