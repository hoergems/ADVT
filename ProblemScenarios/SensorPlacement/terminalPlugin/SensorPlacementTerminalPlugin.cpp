#include <oppt/plugin/Plugin.hpp>
#include "SensorPlacementUserData.hpp"

namespace oppt
{
class SensorPlacementTerminalPlugin: public TerminalPlugin
{
public :
    SensorPlacementTerminalPlugin():
        TerminalPlugin() {

    }

    virtual ~SensorPlacementTerminalPlugin() = default;

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
        auto ud = propagationResult->nextState->getUserData()->as<SensorPlacementUserData>();
        return (ud->reachedGoal or ud->collides);        
    }
};

OPPT_REGISTER_TERMINAL_PLUGIN(SensorPlacementTerminalPlugin)

}




