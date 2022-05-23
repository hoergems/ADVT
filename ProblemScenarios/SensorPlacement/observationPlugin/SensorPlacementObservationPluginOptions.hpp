#ifndef _SENSOR_PLACEMENT_OBSERVATION_PLUGIN_OPTIONS_
#define _SENSOR_PLACEMENT_OBSERVATION_PLUGIN_OPTIONS_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class SensorPlacementObservationPluginOptions: public PluginOptions
{
public:
    SensorPlacementObservationPluginOptions() = default;

    virtual ~SensorPlacementObservationPluginOptions() = default;

    FloatType correctObservationProbability = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPegInHoleObservationPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addPegInHoleObservationPluginOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("observationPluginOptions",
                                     "correctObservationProbability",
                                     &SensorPlacementObservationPluginOptions::correctObservationProbability);
    }

};
}

#endif
