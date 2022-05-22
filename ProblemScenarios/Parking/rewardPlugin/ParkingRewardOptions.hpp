#ifndef _PARKING_REWARD_PLUGIN_OPTIONS_HPP_
#define _PARKING_REWARD_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class ParkingRewardOptions: public PluginOptions
{
public:
    ParkingRewardOptions() = default;

    virtual ~ParkingRewardOptions() = default;

    FloatType stepPenalty = 0.0;

    FloatType exitReward = 0.0; 

    FloatType collisionPenalty = 0.0;   

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addOptions(parser.get());
        return std::move(parser);
    }

    static void addOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("rewardPluginOptions", "stepPenalty", &ParkingRewardOptions::stepPenalty); 
        parser->addOption<FloatType>("rewardPluginOptions", "exitReward", &ParkingRewardOptions::exitReward);
        parser->addOption<FloatType>("rewardPluginOptions", "collisionPenalty", &ParkingRewardOptions::collisionPenalty);        
    }

};
}

#endif
