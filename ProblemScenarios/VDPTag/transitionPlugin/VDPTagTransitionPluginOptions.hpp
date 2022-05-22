#ifndef _VDP_TAG_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _VDP_TAG_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class VDPTagTransitionPluginOptions: public PluginOptions
{
public:
    VDPTagTransitionPluginOptions() = default;

    virtual ~VDPTagTransitionPluginOptions() = default;    

    unsigned int particlePlotLimit = 0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addVDPTagTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addVDPTagTransitionPluginOptions(options::OptionParser* parser) {        
        parser->addOption<unsigned int>("simulation", "particlePlotLimit", &VDPTagTransitionPluginOptions::particlePlotLimit);

    }

};
}

#endif
