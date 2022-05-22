#ifndef _PUSHBOX_OBSERVATION_PLUGIN_OPTIONS_HPP_
#define _PUSHBOX_OBSERVATION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class PushboxObservationPluginOptions: public PluginOptions
{
public:
    PushboxObservationPluginOptions() = default;

    virtual ~PushboxObservationPluginOptions() = default;

    FloatType observationUncertainty = 0.0;

    FloatType positionObservationUncertainty = 0.0;

    unsigned int numberOfObservationBuckets = 0;

    bool usePositionObservation = false;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPushboxObservationOptions(parser.get());
        return std::move(parser);
    }

    static void addPushboxObservationOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("observationPluginOptions",
                                     "observationUncertainty",
                                     &PushboxObservationPluginOptions::observationUncertainty);
        parser->addOptionWithDefault<FloatType>("observationPluginOptions",
                                     "positionObservationUncertainty",
                                     &PushboxObservationPluginOptions::positionObservationUncertainty, 0.0);
        parser->addOption<unsigned int>("observationPluginOptions",
                                        "observationBuckets",
                                        &PushboxObservationPluginOptions::numberOfObservationBuckets);
        parser->addOption<bool>("observationPluginOptions",
                                "usePositionObservation",
                                &PushboxObservationPluginOptions::usePositionObservation);
    }

};
}

#endif
