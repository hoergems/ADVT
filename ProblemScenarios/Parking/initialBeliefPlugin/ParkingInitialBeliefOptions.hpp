#ifndef _PARKING_INITIAL_BELIEF_OPTIONS_HPP_
#define _PARKING_INITIAL_BELIEF_OPTIONS_HPP_s
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class ParkingInitialBeliefOptions: public PluginOptions
{
public:
    ParkingInitialBeliefOptions() = default;

    virtual ~ParkingInitialBeliefOptions() = default;

    FloatType positionError = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addOptions(parser.get());
        return std::move(parser);
    }

    static void addOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("initialBeliefOptions", "positionError", &ParkingInitialBeliefOptions::positionError);
    }

};
}

#endif
