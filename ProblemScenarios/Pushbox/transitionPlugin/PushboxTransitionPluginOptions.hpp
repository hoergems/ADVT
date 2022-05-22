#ifndef _PUSHBOX_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _PUSHBOX_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class PushboxTransitionPluginOptions: public PluginOptions
{
public:
    PushboxTransitionPluginOptions() = default;

    virtual ~PushboxTransitionPluginOptions() = default;

    FloatType actionUncertainty = 0.0;

    FloatType boxSpeedUncertainty = 0.0;

    FloatType boxPositionMoveUncertainty = 0.0;

    FloatType moveUncertainty = 0.0;

    VectorFloat goalPosition;

    FloatType goalRadius = 0.0;

    unsigned int particlePlotLimit = 0;

    std::string goalLink = "";
    size_t sizeX = 0;
    size_t sizeY = 0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPushboxTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addPushboxTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "actionUncertainty",
                                     &PushboxTransitionPluginOptions::actionUncertainty);
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "boxSpeedUncertainty",
                                     &PushboxTransitionPluginOptions::boxSpeedUncertainty);
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "boxPositionMoveUncertainty",
                                     &PushboxTransitionPluginOptions::boxPositionMoveUncertainty);
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "moveUncertainty",
                                     &PushboxTransitionPluginOptions::moveUncertainty);
        parser->addOption<size_t>("map", "sizeX", &PushboxTransitionPluginOptions::sizeX);
        parser->addOption<size_t>("map", "sizeY", &PushboxTransitionPluginOptions::sizeY);
        parser->addOption<VectorFloat>("map", "goalPosition", &PushboxTransitionPluginOptions::goalPosition);
        parser->addOption<FloatType>("map", "goalRadius", &PushboxTransitionPluginOptions::goalRadius);
        parser->addOption<unsigned int>("simulation", "particlePlotLimit", &PushboxTransitionPluginOptions::particlePlotLimit);

    }

};
}

#endif
