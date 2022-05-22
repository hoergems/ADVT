#ifndef _PARKING_HEURISTIC_PLUGIN_OPTIONS_HPP_
#define _PARKING_HEURISTIC_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class ParkingHeuristicOptions: public PluginOptions
{
public:
    ParkingHeuristicOptions() = default;

    virtual ~ParkingHeuristicOptions() = default; 

    FloatType stepPenalty = 0.0;

    FloatType exitReward = 0.0; 

    FloatType discountFactor = 0.0;  

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addOptions(parser.get());
        return std::move(parser);
    }

    static void addOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("rewardPluginOptions", "stepPenalty", &ParkingHeuristicOptions::stepPenalty);       
        parser->addOption<FloatType>("rewardPluginOptions", "exitReward", &ParkingHeuristicOptions::exitReward);
        parser->addOption<FloatType>("problem", "discountFactor", &ParkingHeuristicOptions::discountFactor);       

    }

};
}

#endif