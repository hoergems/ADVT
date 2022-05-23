#ifndef _SENSOR_PLACEMENT_INITIAL_BELIEF_OPTIONS_HPP_
#define _SENSOR_PLACEMENT_INITIAL_BELIEF_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class SensorPlacementInitialBeliefOptions: public PluginOptions
{
public:
    SensorPlacementInitialBeliefOptions() = default;

    virtual ~SensorPlacementInitialBeliefOptions() = default;

    VectorFloat initialStateVec;

    VectorFloat lowerUpperBound;      

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPegInHoleInitialBeliefOptions(parser.get());
        return std::move(parser);
    }

    static void addPegInHoleInitialBeliefOptions(options::OptionParser* parser) {
        parser->addOption<VectorFloat>("initialBeliefOptions", "initialStateVec", &SensorPlacementInitialBeliefOptions::initialStateVec);
        parser->addOption<VectorFloat>("initialBeliefOptions", "lowerUpperBound", &SensorPlacementInitialBeliefOptions::lowerUpperBound);                
    }

};
}

#endif
