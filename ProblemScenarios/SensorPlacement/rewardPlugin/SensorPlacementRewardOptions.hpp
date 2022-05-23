#ifndef _SENSOR_PLACEMENT_REWARD_OPTIONS_HPP_
#define _SENSOR_PLACEMENT_REWARD_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class SensorPlacementRewardOptions: public PluginOptions
{
public:
    SensorPlacementRewardOptions() = default;

    virtual ~SensorPlacementRewardOptions() = default;

    FloatType stepPenalty = 0.0;

    FloatType illegalMovePenalty = 0.0;

    FloatType exitReward = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPegInHoleRewardOptions(parser.get());
        return std::move(parser);
    }

    static void addPegInHoleRewardOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("rewardPluginOptions", "stepPenalty", &SensorPlacementRewardOptions::stepPenalty);
        parser->addOption<FloatType>("rewardPluginOptions", "illegalMovePenalty", &SensorPlacementRewardOptions::illegalMovePenalty);
        parser->addOption<FloatType>("rewardPluginOptions", "exitReward", &SensorPlacementRewardOptions::exitReward);

    }

};
}

#endif
