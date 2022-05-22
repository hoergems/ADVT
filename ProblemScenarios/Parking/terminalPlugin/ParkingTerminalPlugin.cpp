#include <oppt/plugin/Plugin.hpp>
#include "VehicleUserData.hpp"

namespace oppt
{
class ParkingTerminalPlugin: public TerminalPlugin
{
public :
    ParkingTerminalPlugin():
        TerminalPlugin() {
    }

    virtual ~ParkingTerminalPlugin() = default;

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
        auto ud = propagationResult->nextState->getUserData()->as<VehicleUserData>();
        if (ud->collides or ud->insideGoalArea) {
            return true;
        }
        return false;
    }
};

OPPT_REGISTER_TERMINAL_PLUGIN(ParkingTerminalPlugin)

}




