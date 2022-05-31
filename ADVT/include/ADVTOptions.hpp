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
        parser->addOption<unsigned int>("ADVT", "numEpisodes", &ADVTOptions::numEpisodes);
        parser->addOption<unsigned int>("ADVT", "minParticleCount", &ADVTOptions::minParticleCount);
        parser->addOption<FloatType>("ADVT", "maxObservationDistance", &ADVTOptions::maxObservationDistance);
        parser->addOption<FloatType>("ADVT", "explorationFactor", &ADVTOptions::explorationFactor);
        parser->addOption<FloatType>("ADVT", "explorationFactorDiameter", &ADVTOptions::explorationFactorDiameter);        
        parser->addOption<FloatType>("ADVT", "splitExplorationFactor", &ADVTOptions::splitExplorationFactor);
        parser->addOption<FloatType>("ADVT", "minimumSplittingDiam", &ADVTOptions::minimumSplittingDiam);
        parser->addOption<unsigned int>("ADVT", "numDiameterSamples", &ADVTOptions::numDiameterSamples);
        parser->addOption<bool>("ADVT", "resetTree", &ADVTOptions::resetTree);
        parser->addOptionWithDefault<unsigned int>("ADVT", "maximumDepth", &ADVTOptions::maximumDepth, 0);
        parser->addOption<bool>("ADVT", "rejectionSampling", &ADVTOptions::rejectionSampling);
        parser->addOptionWithDefault<bool>("ADVT", "bellmanBackup", &ADVTOptions::bellmanBackup, true);
        parser->addOption<std::string>("ADVT", "partitioningMode", &ADVTOptions::partitioningMode);
        parser->addOptionWithDefault<std::string>("ADVT", "distanceMeasure", &ADVTOptions::distanceMeasure, "euclidean");
    }
};
}

#endif
