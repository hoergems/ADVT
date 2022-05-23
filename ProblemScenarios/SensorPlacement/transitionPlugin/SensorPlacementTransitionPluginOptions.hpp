#ifndef _SENSOR_PLACEMENT_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _SENSOR_PLACEMENT_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class SensorPlacementTransitionPluginOptions: public PluginOptions
{
public:
    SensorPlacementTransitionPluginOptions() = default;

    virtual ~SensorPlacementTransitionPluginOptions() = default;

    std::string endEffectorLink = "";

    VectorFloat lowerTransitionErrorBound;

    VectorFloat upperTransitionErrorBound;

    VectorString jointNames;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPegInHoleTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addPegInHoleTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("transitionPluginOptions", "endEffectorLink", &SensorPlacementTransitionPluginOptions::endEffectorLink);
        parser->addOption<VectorFloat>("transitionPluginOptions", "lowerTransitionErrorBound", &SensorPlacementTransitionPluginOptions::lowerTransitionErrorBound);
        parser->addOption<VectorFloat>("transitionPluginOptions", "upperTransitionErrorBound", &SensorPlacementTransitionPluginOptions::upperTransitionErrorBound);
        parser->addOption<VectorString>("state", "jointPositions", &SensorPlacementTransitionPluginOptions::jointNames);

    }

};
}

#endif
