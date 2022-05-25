#include <oppt/plugin/Plugin.hpp>
#include "VDPTagState.hpp"
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "VDPTagTransitionPluginOptions.hpp"
#include "VDPTagUserData.hpp"
#include "VDPTagDynamics.hpp"
#include "Utils.hpp"

namespace oppt
{

namespace barriers {
class Barrier {
    VectorFloat start;
    FloatType length;
};
};


class VDPTagTransitionPlugin: public TransitionPlugin
{
public :
    VDPTagTransitionPlugin():
        TransitionPlugin() {

    }

    virtual ~VDPTagTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<VDPTagTransitionPluginOptions>(optionsFile);
        Vector2f c1(1.0, 0.0);
        Vector2f c2(0.0, 1.0);
        Vector2f c3(-1.0, 0.0);
        Vector2f c4(0.0, -1.0);

        cardinals_.push_back(c1);
        cardinals_.push_back(c2);
        cardinals_.push_back(c3);
        cardinals_.push_back(c4);

        numIntegrationSteps_ = int(round(stepSize_ / dt_));

        normalDistr_ = std::make_unique<std::normal_distribution<FloatType>>(0.0, 1.0);

        agentLink_ = getLinkPtr(robotEnvironment_, "Agent::AgentLink");
        targetLink_ = getLinkPtr(robotEnvironment_, "Target::TargetLink");

        if (agentLink_ == nullptr)
            ERROR("Agent lin can't be found");
        if (targetLink_ == nullptr)
            ERROR("Target link can't be found");
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        auto currentState = propagationRequest->currentState;
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        VectorFloat currentStateVec = currentState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();

        Vector2f delta(speed * stepSize_ * cos(actionVec[0]), speed * stepSize_ * sin(actionVec[0]));

        Vector2f nextTargetPos =
            computeTargetPosDeterministic(currentState->as<VDPTagState>()->targetPos(), numIntegrationSteps_, mu_, dt_, dtHalf_, dtDiv6_) +
            posStd_ * Vector2f((*(normalDistr_.get()))(*randomEngine), (*(normalDistr_.get()))(*randomEngine));
        
        Vector2f nextAgentPos =
            barrierStop(currentState->as<VDPTagState>()->agentPos(),
                        speed * stepSize_ * rotate(Vector2f(1, 0), actionVec[0]),
                        cardinals_);
        PropagationResultSharedPtr propRes(new PropagationResult);
        propRes->nextState =
            RobotStateSharedPtr(new VDPTagState(nextAgentPos, nextTargetPos));

        OpptUserDataSharedPtr ud(new VDPTagUserData);
        ud->as<VDPTagUserData>()->dist = (nextTargetPos - nextAgentPos).norm();
        ud->as<VDPTagUserData>()->activeBeam = activeBeam(nextTargetPos - nextAgentPos);
        
        propRes->nextState->setUserData(ud);
        propRes->action = propagationRequest->action;
        if (robotEnvironment_->isExecutionEnvironment() or
                (static_cast<const VDPTagTransitionPluginOptions *>(options_.get())->particlePlotLimit > 0)) {
            geometric::Pose agentPose(propRes->nextState->as<VDPTagState>()->agentPos()[0],
                                      propRes->nextState->as<VDPTagState>()->agentPos()[1],
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0);
            geometric::Pose targetPose(nextTargetPos[0],
                                       nextTargetPos[1],
                                       0.0,
                                       0.0,
                                       0.0,
                                       0.0);
            agentLink_->SetWorldPose(agentPose.toGZPose());
            targetLink_->SetWorldPose(targetPose.toGZPose());
            propRes->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
        }

        return propRes;
    }

private:
    FloatType mu_ = 2.0;

    FloatType speed = 1.0;

    FloatType dt_ = 0.1;

    const FloatType dtHalf_ = dt_ / 2.0;

    const FloatType dtDiv6_ = dt_ / 6.0;

    FloatType stepSize_ = 0.5;

    FloatType posStd_ = 0.05;

    int numIntegrationSteps_ = 0;

    std::vector<Vector2f> cardinals_;    

    std::unique_ptr<std::normal_distribution<FloatType>> normalDistr_;

    gazebo::physics::Link* agentLink_ = nullptr;

    gazebo::physics::Link* targetLink_ = nullptr;

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

    VectorFloat toStdVec(const Vector2f &vec) const {
        return VectorFloat({vec[0], vec[1]});
    }

    Vector2f rotate(const Vector2f &vec, const FloatType &angle) const {
        float sinn = sin(angle);
        float coss = cos(angle);

        Vector2f rotated;
        rotated.x() = (coss * vec.x()) - (sinn * vec.y());
        rotated.y() = (sinn * vec.x()) + (coss * vec.y());
        return rotated;
    }
};

OPPT_REGISTER_TRANSITION_PLUGIN(VDPTagTransitionPlugin)

}