#ifndef _PUSHBOX_INTIAL_BELIEF_PLUGIN_HPP_
#define _PUSHBOX_INTIAL_BELIEF_PLUGIN_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class PushboxInitialBeliefOptions: public PluginOptions
{
public:
    PushboxInitialBeliefOptions() = default;

    virtual ~PushboxInitialBeliefOptions() = default;

    VectorFloat initialStateVec;

    FloatType initialBoxPositionUncertainty = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPushboxInitialBeliefOptions(parser.get());
        return std::move(parser);
    }

    static void addPushboxInitialBeliefOptions(options::OptionParser* parser) {
        parser->addOption<VectorFloat>("initialBeliefOptions", "initialStateVec", &PushboxInitialBeliefOptions::initialStateVec);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "initialBoxPositionUncertainty",
                                     &PushboxInitialBeliefOptions::initialBoxPositionUncertainty);
    }

};
}

#endif
