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

        normalDistr_ = std::make_unique<std::normal_distribution<FloatType>>(0.0, 1.0);
        cardinals_.push_back(Vector2f(1, 0));
        cardinals_.push_back(Vector2f(0, 1));
        cardinals_.push_back(Vector2f(-1, 0));
        cardinals_.push_back(Vector2f(0, -1));

        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        //return 0.0;
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        ActionSharedPtr act;
        FloatType reward = 0.0;
        FloatType currentDiscount = 1.0;
        FloatType totalDiscountedReward = 0.0;
        int steps = numRolloutSteps_ - heuristicInfo->currentStep;
        auto agentPos = heuristicInfo->currentState->as<VDPTagState>()->agentPos();
        auto targetPos = heuristicInfo->currentState->as<VDPTagState>()->targetPos();
        for (size_t i = 0; i != steps; ++i) {
            auto action = randomAction();
            agentPos = barrierStop(agentPos,
                                   speed_ * stepSize_ * rotate(Vector2f(1, 0), action[0]),
                                   cardinals_);
            targetPos = computeTargetPosDeterministic(targetPos,
                        numIntegrationSteps_,
                        mu_,
                        dt_,
                        dtHalf_,
                        dtDiv6_) +
                        posStd_ * Vector2f((*(normalDistr_.get()))(*randomEngine), (*(normalDistr_.get()))(*randomEngine));
            auto dist = (targetPos - agentPos).norm();
            if (dist <= 0.1) {                
                reward = 100.0;
            } else {
                reward = -1.0;
            }

            if (action[1] > 0.5)
                reward -= 5.0;

            totalDiscountedReward += currentDiscount * reward;
            currentDiscount *= discountFactor_;

            if (dist <= 0.1) {
                break;
            }
        }

        return totalDiscountedReward;
    }

private:
    FloatType discountFactor_ = 0.95;

    const int numRolloutSteps_ = 10;

    std::unique_ptr<std::uniform_real_distribution<FloatType>> actionDistr_ = nullptr;

    std::unique_ptr<std::bernoulli_distribution> actionDistr2_ = nullptr;

    int numIntegrationSteps_ = 0;

    FloatType mu_ = 2.0;

    FloatType speed_ = 1.0;

    FloatType stepSize_ = 0.5;

    FloatType dt_ = 0.1;

    const FloatType dtHalf_ = dt_ / 2.0;

    const FloatType dtDiv6_ = dt_ / 6.0;

    FloatType posStd_ = 0.05;

    std::vector<Vector2f> cardinals_;

    std::unique_ptr<std::normal_distribution<FloatType>> normalDistr_;

private:
    VectorFloat randomAction() const {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        VectorFloat randomAction({(*(actionDistr_.get()))(*randomEngine),
                                  (*(actionDistr2_.get()))(*randomEngine) ? 1.0 : 0.0
                                 });
        return randomAction;
    }

    Vector2f rotate(const Vector2f &vec, const FloatType &angle) const {
        float c = cosf(angle);
        float s = sinf(angle);
        Vector2f rotated;
        rotated.x() = vec.x() * c - vec.y() * s;
        rotated.y() = vec.x() * s + vec.y() * c;
        return rotated;
    }
};

OPPT_REGISTER_HEURISTIC_PLUGIN(VDPTagHeuristicPlugin)

}


