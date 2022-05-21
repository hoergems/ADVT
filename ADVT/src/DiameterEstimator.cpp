#include "DiameterEstimator.hpp"
#include "Partition.hpp"
#include <oppt/global.hpp>
#include <oppt/robotHeaders/Action.hpp>
#include "BoundingSphere.hpp"

namespace oppt {
DiameterEstimator::DiameterEstimator():
	normalDistribution_(new std::normal_distribution<FloatType>(0.0, 1.0)),
	uniformDistribution_(new std::uniform_real_distribution<FloatType>(0.0, 1.0)) {
}

FloatType DiameterEstimator::estimateDiameter(RandomEngine *randomEngine,
                const Partition *partition,
                const DistanceMeasure *distanceMeasure,
                const unsigned int &numBoundaryPoints,
                BoundaryPoints &currentBoundaryPoints,
                const Vectordf &voronoiCenter,
                const FloatType &rootDiameter) const {
	if (currentBoundaryPoints.size() < numBoundaryPoints) {
		auto numAdditionalPoints = numBoundaryPoints - currentBoundaryPoints.size();
		std::vector<Vectordf> pointsOnSphere =
		        samplePointPairsFromOutsideActionSpace((unsigned int)(numAdditionalPoints / 2), voronoiCenter, randomEngine, rootDiameter);
		currentBoundaryPoints.reserve(numBoundaryPoints);
		for (size_t i = 0; i != pointsOnSphere.size(); ++i) {
			currentBoundaryPoints.push_back(toStdVec<FloatType>(projectOntoBoundary(partition, voronoiCenter, pointsOnSphere[i], distanceMeasure)));
		}
	}

	return boundingSphere::DiamFunctions::diameter(currentBoundaryPoints);
}

Vectordf DiameterEstimator::projectOntoBoundary(const Partition *partition,
                const Vectordf &voronoiCenter,
                const Vectordf &point,
                const DistanceMeasure *distanceMeasure) const {
	Vectordf x1 = voronoiCenter;
	Vectordf x2 = point;
	while ((*distanceMeasure)(x1, x2) > 1e-3) {
		auto xn = (x1 + x2) / 2.0;
		//auto xn = x1 + (x2 - x1) / 2.0;
		if (partition->isInPartition(distanceMeasure, toStdVec<FloatType>(xn))) {
			x1 = xn;
		} else {
			x2 = xn;
		}
	}

	return x1;
}

std::vector<Vectordf> DiameterEstimator::samplePointPairsFromOutsideActionSpace(const unsigned int &numPointPairs,
                const Vectordf &voronoiCenter,
                RandomEngine *randomEngine,
                const FloatType &rootDiameter) const {
	std::vector<Vectordf> points;
	points.reserve(numPointPairs * 2);
	//cout << "numPointPairs: " << numPointPairs << endl;
	if (numPointPairs == 0) {
		// Sample a single point and don't compute its antipolar point
		Vectordf Xs(voronoiCenter.size());
		for (size_t j = 0; j != voronoiCenter.size(); ++j) {
			Xs[j] = (*(normalDistribution_.get()))(*randomEngine);
		}

		FloatType norm = Xs.norm();
		Xs = (10.5 * rootDiameter / norm) * Xs;
		points.push_back(Xs);
	} else {
		for (size_t i = 0; i != numPointPairs; ++i) {
			bool inPartition = true;
			Vectordf Xs(voronoiCenter.size());
			Vectordf Xs2(voronoiCenter.size());
			for (size_t j = 0; j != voronoiCenter.size(); ++j) {
				Xs[j] = (*(normalDistribution_.get()))(*randomEngine);
			}

			FloatType norm = Xs.norm();
			Xs = ((rootDiameter + 1.0) / norm) * Xs;
			Xs2 = -1.0 * (voronoiCenter + Xs);
			points.push_back(Xs);
			points.push_back(Xs2);

		}
	}

	return points;
}

std::vector<VectorFloat> DiameterEstimator::sampleUniform(const unsigned int &numSamples,
                const Partition *partition,
                const DistanceMeasure *distanceMeasure,
                RandomEngine * randomEngine,
                const FloatType & rootDiameter) const {
	Vectordf voronoiCenterEigen = toEigenVec(partition->sampleAction(randomEngine)->as<VectorAction>()->asVector());
	std::vector<VectorFloat> samples;
	samples.reserve(numSamples);

	for (size_t i = 0; i != numSamples; ++i) {
		//Perform a random walk
		Vectordf x = voronoiCenterEigen;
		for (size_t j = 0; j != 10; ++j) {
			//for (size_t j = 0; j != 50; ++j) {
			Vectordf randomBoundaryPoint =
			        projectOntoBoundary(partition,
			                            x,
			                            samplePointPairsFromOutsideActionSpace(1, x, randomEngine, rootDiameter)[0],
			                            distanceMeasure);
			//VectorFloat randomBoundaryPoint = boundaryPoints_[d(*randomEngine)];
			Vectordf dir = randomBoundaryPoint - x;
			dir.normalize();
			FloatType dist = (*distanceMeasure)(randomBoundaryPoint, x);

			x = x + (*(uniformDistribution_.get()))(*randomEngine) * dist * dir;
		}

		samples.push_back(toStdVec<FloatType>(x));
	}


	return samples;
}

}