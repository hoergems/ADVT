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
#include "SensorPlacementTransitionPluginOptions.hpp"
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "GoalParser.hpp"
#include "SensorPlacementUserData.hpp"

namespace oppt
{
class SensorPlacementTransitionPlugin: public TransitionPlugin
{
public :
    SensorPlacementTransitionPlugin():
        TransitionPlugin() {

    }

    virtual ~SensorPlacementTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementTransitionPluginOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementTransitionPluginOptions *>(options_.get());
        auto re = robotEnvironment_->getRobot()->getRandomEngine();

        eeLinkName_ = options->endEffectorLink;
        eeLink_ = getLinkPtr(robotEnvironment_, eeLinkName_);
        if (eeLink_ == nullptr)
            ERROR("EE link link couldn't be found");

        parseGoal_();

        transitionErrorDistribution_ = 
        std::make_unique<UniformDistribution<FloatType>>(options->lowerTransitionErrorBound, options->upperTransitionErrorBound, re);

        for (auto &jointName: options->jointNames) {
            joints.push_back(getJointPtr(robotEnvironment_, jointName));
        }


        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        PropagationResultSharedPtr propRes(new PropagationResult);

        robotEnvironment_->getGazeboInterface()->setWorldState(propagationRequest->currentState->getGazeboWorldState().get());

        VectorFloat currentStateVec = propagationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat nextStateVec = oppt::addVectors(currentStateVec, propagationRequest->action->as<VectorAction>()->asVector());


        // Sample transitionError
        nextStateVec = oppt::addVectors(nextStateVec, toStdVec<FloatType>(transitionErrorDistribution_->sample(1).col(0)));        
        robotEnvironment_->getGazeboInterface()->setStateVector(nextStateVec);
        for (size_t i = 0; i != nextStateVec.size(); ++i) {
            nextStateVec[i] = joints[i]->Position(0);
        }

        propRes->nextState = RobotStateSharedPtr(new VectorState(nextStateVec));
        propRes->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
        auto userData = makeUserData_(propagationRequest->currentState, propRes->nextState);
        propRes->nextState->setUserData(userData);
        return propRes;
    }

private:
    std::string eeLinkName_ = "";

    gazebo::physics::Link* eeLink_ = nullptr;

    VectorFloat goalPosition_;

    FloatType goalRadius_ = 0.0;

    std::unique_ptr<UniformDistribution<FloatType>> transitionErrorDistribution_ = nullptr;    

    std::vector<gazebo::physics::Joint *> joints;

private:
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

    gazebo::physics::Joint *getJointPtr(const RobotEnvironment *robotEnvironment, const std::string &name) {
        auto joints = robotEnvironment->getGazeboInterface()->getJoints();
        gazebo::physics::Joint *foundJoint = nullptr;
        for (auto &joint : joints) {
            if (joint->GetScopedName() == name) {
                foundJoint = joint;
                break;
            }
        }

        if (!foundJoint) {
            WARNING("Joint '" + name + "' could not be found");
            getchar();
        }

        return foundJoint;
    }

    OpptUserDataSharedPtr makeUserData_(const RobotStateSharedPtr &previousState, const RobotStateSharedPtr &state) const {
        OpptUserDataSharedPtr userData(new SensorPlacementUserData);
        //auto collisionReport = robotEnvironment_->getRobot()->makeDiscreteCollisionReportDirty();
        userData->as<SensorPlacementUserData>()->collides = false;
        userData->as<SensorPlacementUserData>()->eeTouchesWall = false;
        userData->as<SensorPlacementUserData>()->eePose = geometric::Pose(eeLink_->WorldPose());

        if (std::fabs(goalPosition_[0] - eeLink_->WorldPose().Pos().X()) < 0.15)
            userData->as<SensorPlacementUserData>()->eeTouchesWall = true;
        if (eeLink_->WorldPose().Pos().X() > goalPosition_[0] + 0.1)        
            userData->as<SensorPlacementUserData>()->collides = true;

        if (robotEnvironment_->isExecutionEnvironment()) {
            cout << "collides: " << userData->as<SensorPlacementUserData>()->collides << endl;
            cout << "touchesWall: " << userData->as<SensorPlacementUserData>()->eeTouchesWall << endl;
        }   

        // Check if we reached the goal
        VectorFloat eePosition({eeLink_->WorldPose().Pos().X(), eeLink_->WorldPose().Pos().Y(), eeLink_->WorldPose().Pos().Z()});
        userData->as<SensorPlacementUserData>()->reachedGoal = math::euclideanDistance(eePosition, goalPosition_) < goalRadius_ ? true : false;
        return userData;
    }

    void parseGoal_() {
        GoalParser p;
        VectorFloat goalArea = p.parseGoalAreaFromFile(robotEnvironment_->getWorldFile());
        goalPosition_ = VectorFloat({goalArea[0], goalArea[1], goalArea[2]});
        goalRadius_ = goalArea[3];
    }
};

OPPT_REGISTER_TRANSITION_PLUGIN(SensorPlacementTransitionPlugin)

}
