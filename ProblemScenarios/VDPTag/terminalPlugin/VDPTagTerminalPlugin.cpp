#include <oppt/plugin/Plugin.hpp>
#include "VDPTagState.hpp"
#include "VDPTagUserData.hpp"

namespace oppt
{
class VDPTagTerminalPlugin: public TerminalPlugin
{
public :
    VDPTagTerminalPlugin():
        TerminalPlugin() {
    }

    virtual ~VDPTagTerminalPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
        ValidityReportSharedPtr validityReport(new ValidityReport(propagationResult->nextState));
        validityReport->isValid = true;
        validityReport->satisfiesConstraints = true;
        validityReport->collided = false;
        return validityReport;
    }

    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {
        FloatType dist =
            (propagationResult->nextState->as<VDPTagState>()->targetPos() - propagationResult->nextState->as<VDPTagState>()->agentPos()).norm();        
        if (dist < tagRadius_)
            return true;
        return false;
    }

private:
    FloatType tagRadius_ = 0.1;
};

OPPT_REGISTER_TERMINAL_PLUGIN(VDPTagTerminalPlugin)

}




