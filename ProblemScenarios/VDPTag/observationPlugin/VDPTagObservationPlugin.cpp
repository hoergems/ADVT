#include <oppt/plugin/Plugin.hpp>
#include "VDPTagState.hpp"
#include "VDPTagObservation.hpp"
#include "VDPTagUserData.hpp"
#include "Utils.hpp"

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
        distrGood_ = std::make_unique<std::normal_distribution<float>>(0.0, 0.1);
        distrBad_ = std::make_unique<std::normal_distribution<float>>(0.0, 5.0);
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        Vector2f agentPos = observationRequest->currentState->as<VDPTagState>()->agentPos();
        Vector2f targetPos = observationRequest->currentState->as<VDPTagState>()->targetPos();

        //FloatType dist = observationRequest->currentState->getUserData()->as<VDPTagUserData>()->dist;
        FloatType dist = observationRequest->currentState->getUserData()->as<VDPTagUserData>()->dist;
        int aBeam = observationRequest->currentState->getUserData()->as<VDPTagUserData>()->activeBeam;

        VectorFloat obs(8, 0.0);
        bool look = observationRequest->action->as<VectorAction>()->asVector()[1] < 0.5 ? false : true;
        if (look) {
            obs[aBeam] = dist + (*(distrGood_.get()))(*randomEngine);
        } else {
            obs[aBeam] = dist + (*(distrBad_.get()))(*randomEngine);
        }

        for (size_t i = 0; i != 8; ++i) {
            if (i != aBeam) {
                obs[i] = 1.0 + (*(distrBad_.get()))(*randomEngine);
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
        int beamState = state->getUserData()->as<VDPTagUserData>()->activeBeam;

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
    std::unique_ptr<std::normal_distribution<float>> distrGood_ = nullptr;
    std::unique_ptr<std::normal_distribution<float>> distrBad_ = nullptr;

    static constexpr float ACTIVE_MEAS_STD = 0.1f;
    static constexpr float MEAS_STD = 5.0f;


private:
    float normalPDF(const float &x, const float &mu, const float &stdDev) const {
        static const float inv_sqrt_2pi = 0.3989422804014327;
        float a = (x - mu) / stdDev;

        return inv_sqrt_2pi / stdDev * std::exp(-0.5f * a * a);
    }    
};

OPPT_REGISTER_OBSERVATION_PLUGIN(VDPTagObservationPlugin)

}