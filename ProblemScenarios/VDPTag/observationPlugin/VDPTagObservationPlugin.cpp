#include <oppt/plugin/Plugin.hpp>
#include "VDPTagState.hpp"
#include "VDPTagObservation.hpp"
#include "VDPTagUserData.hpp"

namespace oppt
{
class VDPTagObservationPlugin: public ObservationPlugin
{
public :
    VDPTagObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~VDPTagObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        distrGood_ = std::make_unique<std::normal_distribution<FloatType>>(0.0, 0.1);
        distrBad_ = std::make_unique<std::normal_distribution<FloatType>>(0.0, 5.0);
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        Vector2f agentPos = observationRequest->currentState->as<VDPTagState>()->agentPos();
        Vector2f targetPos = observationRequest->currentState->as<VDPTagState>()->targetPos();

        //FloatType dist = observationRequest->currentState->getUserData()->as<VDPTagUserData>()->dist;
        Vector2f relativePos = targetPos - agentPos;
        FloatType dist = relativePos.norm();
        int aBeam = activeBeam(targetPos - agentPos);

        VectorFloat obs(8, 0.0);
        bool look = observationRequest->action->as<VectorAction>()->asVector()[1] < 0.5 ? false : true;
        if (look) {
            obs[aBeam] = std::normal_distribution<float>(dist, ACTIVE_MEAS_STD)(*randomEngine);
        } else {
            obs[aBeam] = std::normal_distribution<float>(dist, MEAS_STD)(*randomEngine);
        }

        for (size_t i = 0; i != 8; ++i) {
            if (i != aBeam) {
                obs[i] = std::normal_distribution<float>(1.0, MEAS_STD)(*randomEngine);
            }
        }
        

        ObservationResultSharedPtr obsRes(new ObservationResult);
        obsRes->observation = ObservationSharedPtr(new VDPTagObservation(obs, aBeam));
        return obsRes;
    }

    virtual double calcLikelihood(const RobotStateSharedPtr &state,
                                  const Action *action,
                                  const Observation *observation) const override {
        VectorFloat observationVec = observation->as<VectorObservation>()->asVector();
        Vector2f relativePos = state->as<VDPTagState>()->targetPos() - state->as<VDPTagState>()->agentPos();
        FloatType dist = relativePos.norm();

        bool looking = action->as<VectorAction>()->asVector()[1] > 0.5 ? true : false;
        int beamState = activeBeam(state->as<VDPTagState>()->targetPos() - state->as<VDPTagState>()->agentPos());
        int beamObservation = observation->as<VDPTagObservation>()->beam();

        FloatType pdff = 1.0;
        if (looking) {
            pdff *= normalPDF(observationVec[beamState], dist, ACTIVE_MEAS_STD);            
        } else {
            pdff *= normalPDF(observationVec[beamState], dist, MEAS_STD);
        }
        for (size_t i = 0; i != 8; ++i) {
            if (i != beamState) {
                pdff *= normalPDF(observationVec[i], 1.0, MEAS_STD);
            }
        }        

        return pdff;
    }

private:
    std::unique_ptr<std::normal_distribution<FloatType>> distrGood_ = nullptr;
    std::unique_ptr<std::normal_distribution<FloatType>> distrBad_ = nullptr;

    static constexpr float ACTIVE_MEAS_STD = 0.1f;
    static constexpr float MEAS_STD = 5.0f;


private:
    float normalPDF(const float &x, const float &mu, const float &stdDev) const {
        static const float inv_sqrt_2pi = 0.3989422804014327;
        float a = (x - mu) / stdDev;

        return inv_sqrt_2pi / stdDev * std::exp(-0.5f * a * a);
    }

    int activeBeam(const Vector2f &relativePos) const {
        Vector2f unit(1.0, 0.0);
        float angle = angleTo(unit, relativePos);

        while (angle <= 0.0f) {
            angle += 2 * M_PI;
        }
        size_t x = static_cast<size_t>(lround(ceilf(8 * angle / (2 * M_PI))) - 1);
        return std::max(static_cast<size_t>(0), std::min(static_cast<size_t>(7), x));       
    }

    float rectifyAngle(const float &angle) const {
        return angle - (ceilf((angle + M_PI) / (2 * M_PI)) - 1) * 2 * M_PI;
    }

    FloatType angleTo(const Vector2f &from, const Vector2f &to) const {
        return rectifyAngle(atan2f(from.x() * to.y() - from.y() * to.x(), from.x() * to.x() + from.y() * to.y()));
    }
};

OPPT_REGISTER_OBSERVATION_PLUGIN(VDPTagObservationPlugin)

}