#ifndef _GPABT_OPTIONS_HPP_
#define _GPABT_OPTIONS_HPP_
#include <oppt/problemEnvironment/ProblemEnvironmentOptions.hpp>

namespace oppt
{
struct ADVTOptions: public oppt::ProblemEnvironmentOptions {
public:
    ADVTOptions() = default;
    virtual ~ADVTOptions() = default;

    unsigned int numEpisodes = 0;

    unsigned int minParticleCount = 0;

    FloatType maxObservationDistance = 0.0;

    FloatType explorationFactor = 0.0;

    FloatType explorationFactorDiameter = 0.0;    

    FloatType splitExplorationFactor = 0.0;

    FloatType minimumSplittingDiam = 0.0;

    unsigned int numDiameterSamples = 2;

    bool resetTree = false;

    unsigned int maximumDepth = 0;

    bool rejectionSampling = false;

    bool bellmanBackup = true;    

    std::string partitioningMode = ""; 

    std::string distanceMeasure = "";

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser =
            ProblemEnvironmentOptions::makeParser(simulating);
        addADVTOptions(parser.get());
        return std::move(parser);
    }

    static void addADVTOptions(options::OptionParser* parser) {
        parser->addOption<unsigned int>("HOOT", "numEpisodes", &ADVTOptions::numEpisodes);
        parser->addOption<unsigned int>("HOOT", "minParticleCount", &ADVTOptions::minParticleCount);
        parser->addOption<FloatType>("HOOT", "maxObservationDistance", &ADVTOptions::maxObservationDistance);
        parser->addOption<FloatType>("HOOT", "explorationFactor", &ADVTOptions::explorationFactor);
        parser->addOption<FloatType>("HOOT", "explorationFactorDiameter", &ADVTOptions::explorationFactorDiameter);        
        parser->addOption<FloatType>("HOOT", "splitExplorationFactor", &ADVTOptions::splitExplorationFactor);
        parser->addOption<FloatType>("HOOT", "minimumSplittingDiam", &ADVTOptions::minimumSplittingDiam);
        parser->addOption<unsigned int>("HOOT", "numDiameterSamples", &ADVTOptions::numDiameterSamples);
        parser->addOption<bool>("HOOT", "resetTree", &ADVTOptions::resetTree);
        parser->addOptionWithDefault<unsigned int>("HOOT", "maximumDepth", &ADVTOptions::maximumDepth, 0);
        parser->addOption<bool>("HOOT", "rejectionSampling", &ADVTOptions::rejectionSampling);
        parser->addOptionWithDefault<bool>("HOOT", "bellmanBackup", &ADVTOptions::bellmanBackup, true);
        parser->addOption<std::string>("HOOT", "partitioningMode", &ADVTOptions::partitioningMode);
        parser->addOptionWithDefault<std::string>("HOOT", "distanceMeasure", &ADVTOptions::distanceMeasure, "euclidean");
    }
};
}

#endif
