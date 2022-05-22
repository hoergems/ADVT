#include <oppt/plugin/Plugin.hpp>
#include "PushboxStateUserData.hpp"

namespace oppt
{
class PushboxTerminalPlugin: public TerminalPlugin
{
public :
    PushboxTerminalPlugin():
        TerminalPlugin() {
    }

    virtual ~PushboxTerminalPlugin() = default;

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
        auto userData = propagationResult->nextState->getUserData()->as<PushboxStateUserData>();
        return (userData->isGoalState or userData->isInCollision);
    }
};

OPPT_REGISTER_TERMINAL_PLUGIN(PushboxTerminalPlugin)

}