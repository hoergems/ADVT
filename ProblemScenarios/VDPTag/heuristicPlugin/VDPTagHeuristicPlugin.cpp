#include <oppt/plugin/Plugin.hpp>
#include "VDPTagState.hpp"
#include "VDPTagUserData.hpp"
#include "VDPTagDynamics.hpp"

namespace oppt
{
class VDPTagHeuristicPlugin: public HeuristicPlugin
{
public:
    VDPTagHeuristicPlugin():
        HeuristicPlugin() {
    }

    virtual ~VDPTagHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        actionDistr_ = std::make_unique<std::uniform_real_distribution<FloatType>>(0.0, 2.0 * M_PI); 
        actionDistr2_ = std::make_unique<std::bernoulli_distribution>(0.5);      
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {        
        PropagationRequestSharedPtr propReq(new PropagationRequest);
        PropagationResultSharedPtr propRes;

        ActionSharedPtr act;
        FloatType reward = 0.0;
        auto currentState = heuristicInfo->currentState;
        FloatType currentDiscount = 1.0;
        FloatType totalDiscountedReward = 0.0;
        for (size_t i = 0; i != numRolloutSteps; ++i) {
            propReq->currentState = currentState;
            act = ActionSharedPtr(new VectorAction(randomAction()));
            propReq->action = act.get();

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

private:
    FloatType discountFactor_ = 0.95;

    int numRolloutSteps = 9;

    std::unique_ptr<std::uniform_real_distribution<FloatType>> actionDistr_ = nullptr;

    std::unique_ptr<std::bernoulli_distribution> actionDistr2_ = nullptr;

private:
    VectorFloat randomAction() const {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        FloatType look = (*(actionDistr2_.get()))(*randomEngine) ? 1.0 : 0.0;
        //bool look = (*(actionDistr2_.get()))(*(randomEngine.get()));
        VectorFloat randomAction({(*(actionDistr_.get()))(*randomEngine), look});
        return randomAction;
    }
};

OPPT_REGISTER_HEURISTIC_PLUGIN(VDPTagHeuristicPlugin)

}


