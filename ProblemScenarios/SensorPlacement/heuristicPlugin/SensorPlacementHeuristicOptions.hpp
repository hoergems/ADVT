#ifndef _SENSOR_PLACEMENT_HEURISTIC_OPTIONS_HPP_
#define _SENSOR_PLACEMENT_HEURISTIC_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class SensorPlacementHeuristicOptions: public PluginOptions
{
public:
    SensorPlacementHeuristicOptions() = default;

    virtual ~SensorPlacementHeuristicOptions() = default;

    FloatType exitReward = 0.0;

    std::string endEffectorLink = "";

    FloatType stepPenalty = 0.0;

    FloatType discountFactor = 0.0;
    
    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPegInHoleHeuristicOptions(parser.get());
        return std::move(parser);
    }

    static void addPegInHoleHeuristicOptions(options::OptionParser* parser) {    
        parser->addOption<FloatType>("rewardPluginOptions", "exitReward", &SensorPlacementHeuristicOptions::exitReward);
        parser->addOption<FloatType>("rewardPluginOptions", "stepPenalty", &SensorPlacementHeuristicOptions::stepPenalty);
        parser->addOption<FloatType>("problem", "discountFactor", &SensorPlacementHeuristicOptions::discountFactor);
        parser->addOption<std::string>("transitionPluginOptions", "endEffectorLink", &SensorPlacementHeuristicOptions::endEffectorLink);
    }

};
}

#endif