#ifndef _PARKING_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _PARKING_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class ParkingTransitionPluginOptions: public PluginOptions
{
public:
    ParkingTransitionPluginOptions() = default;

    virtual ~ParkingTransitionPluginOptions() = default; 

    FloatType controlDuration = 1.0;

    FloatType controlErrorVelocity = 0.0;

    FloatType controlErrorYaw = 0.0;

    FloatType controlErrorElevation = 0.0;

    unsigned int numIntegrationSteps = 0;

    std::string vehicleLink = "";

    unsigned int numInputStepsActions = 0;  

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addOptions(parser.get());
        return std::move(parser);
    }

    static void addOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("transitionPluginOptions", "controlDuration", &ParkingTransitionPluginOptions::controlDuration);
        parser->addOption<std::string>("vehicle", "vehicleLink", &ParkingTransitionPluginOptions::vehicleLink);
        parser->addOption<FloatType>("transitionPluginOptions", "controlErrorVelocity", &ParkingTransitionPluginOptions::controlErrorVelocity);
        parser->addOption<FloatType>("transitionPluginOptions", "controlErrorYaw", &ParkingTransitionPluginOptions::controlErrorYaw);
        parser->addOption<FloatType>("transitionPluginOptions", "controlErrorElevation", &ParkingTransitionPluginOptions::controlErrorElevation);
        parser->addOption<unsigned int>("transitionPluginOptions", "numIntegrationSteps", &ParkingTransitionPluginOptions::numIntegrationSteps);
        
        parser->addOptionWithDefault<unsigned int>("ABT",
                "numInputStepsActions",
                &ParkingTransitionPluginOptions::numInputStepsActions, 2);       
    }

};
}

#endif
