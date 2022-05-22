#ifndef _PUSHBOX_TRANSITION_PLUGIN_HPP_
#define _PUSHBOX_TRANSITION_PLUGIN_HPP_
#include <oppt/plugin/Plugin.hpp>
#include "TruncatedNormal.hpp"
#include "PushboxTransitionPluginOptions.hpp"
#include "PushboxStateUserData.hpp"
#include "Map2d.hpp"
#include <oppt/gazeboInterface/GazeboInterface.hpp>

namespace oppt
{
class PushboxTransitionPlugin: public TransitionPlugin
{
public :
    PushboxTransitionPlugin():
        TransitionPlugin() {
    }

    virtual ~PushboxTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxTransitionPluginOptions>(optionsFile);
        auto options = static_cast<const PushboxTransitionPluginOptions *>(options_.get());
        actionDistribution_ = std::make_unique<TruncatedNormalDistribution>(0.0, options->actionUncertainty);
        boxSpeedFactorDistribution_ = std::make_unique<TruncatedNormalDistribution>(1.0, options->boxSpeedUncertainty);
        boxAbsoluteDistribution_ = std::make_unique<TruncatedNormalDistribution>(0.0, options->boxPositionMoveUncertainty);
        moveDistribution_ = std::make_unique<TruncatedNormalDistribution>(0.0, options->moveUncertainty);

        // Map
        map_ = Map(options->sizeX, options->sizeY, ' ');
        //map_.loadFromFile(options->mapPath);

        if (resources::FileExists("map_free_paper.txt") == false) {
            ERROR("map_free_paper.txt couldn't be found");
        }

        auto mapFile = resources::FindFile("map_free_paper.txt");        
        map_.loadFromFile(mapFile);

        robotLink_ = getLinkPtr(robotEnvironment_, "PushboxRobot::PushboxRobotLink");
        opponentLink_ = getLinkPtr(robotEnvironment_, "PushboxOpponent::PushboxOpponentLink");
        if (robotLink_ == nullptr)
            ERROR("Robot link couldn't be found");
        if (opponentLink_ == nullptr)
            ERROR("Opponent link couldn't be found");

        goalPosition_ = options->goalPosition;
        goalRadius_ = options->goalRadius;

        // Extend the goalPosition by the number of required dimensions
        unsigned int numStateDimensionsHalf = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions() / 2;
        if (numStateDimensionsHalf > 2) {
            VectorFloat numAdditionalGoalPositionDimensions(numStateDimensionsHalf - 2, 0.0);
            goalPosition_.insert(goalPosition_.end(), numAdditionalGoalPositionDimensions.begin(), numAdditionalGoalPositionDimensions.end());
        }
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        return propagateStateGeneral_(propagationRequest);
    }

private:
    std::unique_ptr<TruncatedNormalDistribution> actionDistribution_ = nullptr;

    std::unique_ptr<TruncatedNormalDistribution> boxSpeedFactorDistribution_ = nullptr;

    std::unique_ptr<TruncatedNormalDistribution> boxAbsoluteDistribution_ = nullptr;

    std::unique_ptr<TruncatedNormalDistribution> moveDistribution_ = nullptr;

    Map map_;

    VectorFloat goalPosition_;

    FloatType goalRadius_ = 0.0;

    gazebo::physics::Link* robotLink_ = nullptr;

    gazebo::physics::Link* opponentLink_ = nullptr;

private:
    PropagationResultSharedPtr propagateStateGeneral_(const PropagationRequest* propagationRequest) const {
        PropagationResultSharedPtr propRes(new PropagationResult);
        VectorFloat currentStateVec = propagationRequest->currentState->as<VectorState>()->asVector();
        unsigned int numDims = currentStateVec.size() / 2;
        VectorFloat delta = propagationRequest->action->as<VectorAction>()->asVector();
        VectorFloat nextStateVec(currentStateVec);

        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        for (size_t i = 0; i != numDims; ++i) {
            delta[i] += actionDistribution_->sample(*randomEngine);
        }

        VectorFloat robotPosition(currentStateVec.begin(), currentStateVec.begin() + numDims);
        VectorFloat boxPosition(currentStateVec.begin() + numDims, currentStateVec.begin() + 2 * numDims);

        FloatType speedSquared = 0.0;
        for (size_t i = 0; i != numDims; ++i) {
            speedSquared += delta[i] * delta[i];
        }

        bool pushed = false;
        if (speedSquared > 0) {
            FloatType speed = std::sqrt(speedSquared);

            // first get the vector from the robot position to the box position.
            VectorFloat toBox = subtractVectors(boxPosition, robotPosition);

            // then project it onto the line defined by deltaX, deltaY
            FloatType t = 0.0;
            for (size_t i = 0; i != numDims; ++i) {
                t += toBox[i] * delta[i];
            }

            t /= speedSquared;

            VectorFloat closest = addVectors(robotPosition, scaleVector(delta, t));

            FloatType lineToBoxDistanceSquared = 0.0;
            for (size_t i = 0; i != numDims; ++i) {
                lineToBoxDistanceSquared += square(boxPosition[i] - closest[i]);
            }

            // only proceed if we have a proper contact.
            if (lineToBoxDistanceSquared < 1) {
                // how far is the intersection from the 'closest point'.
                FloatType intersectionDistance = std::sqrt(1 - lineToBoxDistanceSquared);

                // we are only interested in the closer solution there would be a second one with a +.
                FloatType contactT = t - intersectionDistance / speed;
                if ( (0 <= contactT) && (contactT <= 1 ) ) {
                    if (robotEnvironment_->isExecutionEnvironment())
                        WARNING("HAVE CONTACT");
                    // we have a collision.
                    VectorFloat contact = addVectors(robotPosition, scaleVector(delta, contactT));

                    // get the vector describing the collision direction. It is unit length by definition of there being contact.
                    VectorFloat boxDirection = subtractVectors(boxPosition, contact);

                    // the box speed is dependent on how fast we bump and the direction we bump (scalar product).
                    FloatType boxSpeed = 0.0;
                    for (size_t i = 0; i != numDims; ++i) {
                        boxSpeed += delta[i] * boxDirection[i];
                    }

                    // make the box move a bit faster
                    boxSpeed *= 5;
                    // ...and random..
                    boxSpeed *= boxSpeedFactorDistribution_->sample(*randomEngine);

                    // make a new box position
                    for (size_t i = 0; i != numDims; ++i) {
                        nextStateVec[numDims + i] += boxDirection[i] * boxSpeed + boxSpeed * boxAbsoluteDistribution_->sample(*randomEngine);
                    }

                    pushed = true;
                }
            } else {
                if (robotEnvironment_->isExecutionEnvironment())
                    WARNING("HAVE NO CONTACT");
            }
        }

        for (size_t i = 0; i != numDims; ++i) {
            nextStateVec[i] = currentStateVec[i] + delta[i] + moveDistribution_->sample(*randomEngine);
        }



        propRes->nextState = RobotStateSharedPtr(new VectorState(nextStateVec));
        OpptUserDataSharedPtr userData(new PushboxStateUserData);
        userData->as<PushboxStateUserData>()->previousState = propagationRequest->currentState;
        userData->as<PushboxStateUserData>()->isGoalState = isGoalState(nextStateVec);
        userData->as<PushboxStateUserData>()->isInCollision = inCollision(nextStateVec);
        //userData->as<PushboxStateUserData>()->isInCollision = false;
        userData->as<PushboxStateUserData>()->pushed = pushed;
        propRes->nextState->setUserData(userData);

        if (nextStateVec.size() <= 6) {
            if (robotEnvironment_->isExecutionEnvironment() or
                    (static_cast<const PushboxTransitionPluginOptions *>(options_.get())->particlePlotLimit > 0 and propagationRequest->userData != nullptr)) {
                VectorFloat robotPosition2(nextStateVec.begin(), nextStateVec.begin() + nextStateVec.size() / 2);
                VectorFloat opponentPosition2(nextStateVec.begin() + nextStateVec.size() / 2, nextStateVec.end());
                geometric::Pose robotPose;
                geometric::Pose opponentPose;
                unsigned int stateHalfDim = nextStateVec.size() / 2;
                if (stateHalfDim == 2) {
                    robotPose = geometric::Pose(robotPosition2[0], robotPosition2[1], 0.0, 0.0, 0.0, 0.0);
                    opponentPose = geometric::Pose(opponentPosition2[0], opponentPosition2[1], 0.0, 0.0, 0.0, 0.0);
                    robotLink_->SetWorldPose(robotPose.toGZPose());
                    opponentLink_->SetWorldPose(opponentPose.toGZPose());
                } else if (stateHalfDim == 3) {
                    robotPose = geometric::Pose(robotPosition2[0], robotPosition2[1], robotPosition2[2], 0.0, 0.0, 0.0);
                    opponentPose = geometric::Pose(opponentPosition2[0], opponentPosition2[1], opponentPosition2[2], 0.0, 0.0, 0.0);
                    robotLink_->SetWorldPose(robotPose.toGZPose());
                    opponentLink_->SetWorldPose(opponentPose.toGZPose());
                } else {
                    // Nothing to visualize
                }

                propRes->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
            }
        }

        return propRes;
    }

    FloatType square(const FloatType &x) const {
        return x * x;
    }

    bool isGoalState(const VectorFloat &stateVec) const {
        if (stateVec.size() == 4)
            return (lookupInMap(stateVec[2], stateVec[3]) == 'g');
        VectorFloat opponentPosition({stateVec[stateVec.size() / 2], stateVec[stateVec.size() / 2 + 1]});
        return math::euclideanDistance(opponentPosition, goalPosition_) <= goalRadius_;
    };

    bool inCollision(const VectorFloat &stateVec) const {
        VectorFloat robotPosition(stateVec.begin(), stateVec.begin() + stateVec.size() / 2);
        VectorFloat opponentPosition(stateVec.begin() + stateVec.size() / 2, stateVec.end());
        if (lookupInMap(robotPosition[0], robotPosition[1]) == '#') return true;
        if (lookupInMap(opponentPosition[0], opponentPosition[1]) == '#') return true;
        return false;
    }

    FloatType euclideanDistanceSquaredTo(const VectorFloat &stateVec) const {
        FloatType a = stateVec[0] - stateVec[2];
        FloatType b = stateVec[1] - stateVec[3];
        return a * a + b * b;
    }

    char lookupInMap(const FloatType &x, const FloatType &y) const {
        if ( (x < 0) || (y < 0) ) return '#';
        size_t x_ = std::floor(x);
        size_t y_ = std::floor(y);
        if ( (x_ >= map_.sizeX()) || (y_ >= map_.sizeY()) ) return '#';
        return map_(x_, y_);
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
};

OPPT_REGISTER_TRANSITION_PLUGIN(PushboxTransitionPlugin)

}

#endif
