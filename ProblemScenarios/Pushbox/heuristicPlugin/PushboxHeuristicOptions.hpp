#ifndef _PUSHBOX_HEURISTIC_OPTIONS_HPP_
#define _PUSHBOX_HEURISTIC_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class PushboxHeuristicOptions: public PluginOptions
{
public:
    PushboxHeuristicOptions() = default;

    virtual ~PushboxHeuristicOptions() = default;

    FloatType moveCost = 0.0;

    FloatType goalReward = 0.0;

    FloatType collisionPenalty = 0.0;

    VectorFloat goalPosition;

    FloatType discountFactor = 0.0;  

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPushboxHeuristicOptions(parser.get());
        return std::move(parser);
    }

    static void addPushboxHeuristicOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("rewardPluginOptions", "goalReward", &PushboxHeuristicOptions::goalReward);
        parser->addOption<FloatType>("rewardPluginOptions", "moveCost", &PushboxHeuristicOptions::moveCost);
        parser->addOption<FloatType>("rewardPluginOptions", "collisionPenalty", &PushboxHeuristicOptions::collisionPenalty);
        parser->addOption<VectorFloat>("map", "goalPosition", &PushboxHeuristicOptions::goalPosition);
        parser->addOption<FloatType>("problem", "discountFactor", &PushboxHeuristicOptions::discountFactor);

    }

};
}

#endif