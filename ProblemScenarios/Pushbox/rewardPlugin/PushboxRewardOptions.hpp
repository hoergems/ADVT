#ifndef _PUSHBOX_REWARD_PLUGIN_OPTIONS_HPP_
#define _PUSHBOX_REWARD_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class PushboxRewardOptions: public PluginOptions
{
public:
    PushboxRewardOptions() = default;

    virtual ~PushboxRewardOptions() = default;

    FloatType moveCost = 0.0;
    FloatType goalReward = 0.0;
    FloatType collisionPenalty = 0.0;
    
    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();        
        addPushboxRewardOptions(parser.get());
        cout << "added options" << endl;
        return std::move(parser);
    }

    static void addPushboxRewardOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("rewardPluginOptions", "moveCost", &PushboxRewardOptions::moveCost);
        parser->addOption<FloatType>("rewardPluginOptions", "goalReward", &PushboxRewardOptions::goalReward);
        parser->addOption<FloatType>("rewardPluginOptions", "collisionPenalty", &PushboxRewardOptions::collisionPenalty);
    }

};
}

#endif
