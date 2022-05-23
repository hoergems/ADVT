#ifndef _PARTITION_HPP_
#define _PARTITION_HPP_
#include <oppt/opptCore/core.hpp>
#include "TreeElement.hpp"
#include "DiameterEstimator.hpp"
#include "ADVTOptions.hpp"
#include "DistanceMeasure.hpp"


namespace oppt {
class PartitionAgent;
class PartitionTreeSerializer;
class Partition: public TreeElement {
public:
	friend class PartitionAgent;
	friend class PartitionTreeSerializer;
	Partition(TreeElement *const parentPartition,
	          const std::shared_ptr<Action> &action,
	          const ProblemEnvironmentOptions *options);

	virtual ~Partition() = default;

	_NO_COPY_BUT_MOVE(Partition)

	_STATIC_CAST

	virtual size_t getID() const;

	virtual void print() const override;

	virtual Action *sampleAction(RandomEngine *randomEngine) const = 0;

	virtual std::pair<TreeElement*, TreeElement*> split(const DiameterEstimator *diameterEstimator,
	        const DistanceMeasure *distanceMeasure,
	        RandomEngine *randomEngine) = 0;

	//virtual void computeDiameter(RandomEngine *randomEngine) const = 0;

	virtual FloatType getDiameter(const DiameterEstimator *diameterEstimator,
	                              const DistanceMeasure *distanceMeasure,
	                              RandomEngine *randomEngine) const = 0;

	virtual bool isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const = 0;

	virtual bool isSplittable(const FloatType &rootDiameter) const = 0;

	//const DistanceMeasure *getDistanceMeasure() const;

	virtual void serializePartition(std::ofstream &os,
	                                const DiameterEstimator *diameterEstimator,
	                                const DistanceMeasure *distanceMeasure,
	                                RandomEngine *randomEngine) = 0;

protected:
	size_t id_ = 0;

	std::shared_ptr<Action> action_ = nullptr;

	mutable FloatType diameter_ = std::numeric_limits<FloatType>::quiet_NaN();

	const ProblemEnvironmentOptions *options_;

	//const DistanceMeasure *distanceMeasure_ = nullptr;

};

class RectanglePartition: public Partition {
public:
	RectanglePartition(TreeElement *const parentPartition,
	                   const std::shared_ptr<Action> &action,
	                   const VectorFloat &lowerBound,
	                   const VectorFloat &upperBound,
	                   const ProblemEnvironmentOptions *options);

	~RectanglePartition() = default;

	_NO_COPY_BUT_MOVE(RectanglePartition)

	VectorFloat lower_;

	VectorFloat upper_;

	virtual Action *sampleAction(RandomEngine *randomEngine) const override;

	virtual std::pair<TreeElement*, TreeElement*> split(const DiameterEstimator *diameterEstimator,
	        const DistanceMeasure *distanceMeasure,
	        RandomEngine *randomEngine) override;

	//virtual void computeDiameter(RandomEngine *randomEngine) const override;

	virtual FloatType getDiameter(const DiameterEstimator *diameterEstimator,
	                              const DistanceMeasure *distanceMeasure,
	                              RandomEngine *randomEngine) const override;

	virtual bool isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const override;

	virtual bool isSplittable(const FloatType &rootDiameter) const override;

	virtual void serializePartition(std::ofstream &os,
	                                const DiameterEstimator *diameterEstimator,
	                                const DistanceMeasure *distanceMeasure,
	                                RandomEngine *randomEngine) override;

private:
	VectorFloat sampleUniformlyFromPartition_(RandomEngine *randomEngine) const;

};

class VoronoiPartition: public Partition {
public:
	VoronoiPartition(const std::shared_ptr<Action> &action,
	                 TreeElement *const parentPartition,
	                 const VectorFloat *lowerActionBounds,
	                 const VectorFloat *upperActionBounds,
	                 const FloatType &rootDiameter,
	                 const ProblemEnvironmentOptions *options);

	virtual ~VoronoiPartition() = default;

	_NO_COPY_BUT_MOVE(VoronoiPartition)

	virtual Action *sampleAction(RandomEngine *randomEngine) const override;

	virtual std::pair<TreeElement*, TreeElement*> split(const DiameterEstimator *diameterEstimator,
	        const DistanceMeasure *distanceMeasure,
	        RandomEngine *randomEngine) override;

	//virtual void computeDiameter(RandomEngine *randomEngine) const override;

	virtual FloatType getDiameter(const DiameterEstimator *diameterEstimator,
	                              const DistanceMeasure *distanceMeasure,
	                              RandomEngine *randomEngine) const override;

	virtual bool isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const override;

	virtual bool isSplittable(const FloatType &rootDiameter) const override;

	virtual void serializePartition(std::ofstream &os, const DiameterEstimator *diameterEstimator,
	                                const DistanceMeasure *distanceMeasure,
	                                RandomEngine *randomEngine) override;

protected:
	const VectorFloat *lowerActionBounds_;

	const VectorFloat *upperActionBounds_;

	mutable BoundaryPoints boundaryPoints_;

	FloatType rootDiameter_ = 0.0;

protected:
	VectorFloat sampleUniformlyFromPartition_(const DiameterEstimator *diameterEstimator,
	        const DistanceMeasure *distanceMeasure,
	        RandomEngine *randomEngine) const;
};

class LineSegmentPartition: public Partition {
public:
	LineSegmentPartition(const std::shared_ptr<Action> &action,
	                     TreeElement *const parentPartition,
	                     const FloatType &l,
	                     const FloatType &r,
	                     const ProblemEnvironmentOptions *options);

	virtual ~LineSegmentPartition() = default;

	_NO_COPY_BUT_MOVE(LineSegmentPartition)

	const FloatType getL() const;

	const FloatType getR() const;

	virtual Action *sampleAction(RandomEngine *randomEngine) const override;

	virtual std::pair<TreeElement*, TreeElement*> split(const DiameterEstimator *diameterEstimator,
	        const DistanceMeasure *distanceMeasure,
	        RandomEngine *randomEngine) override;

	//virtual void computeDiameter(RandomEngine *randomEngine) const override;

	virtual FloatType getDiameter(const DiameterEstimator *diameterEstimator,
	                              const DistanceMeasure *distanceMeasure,
	                              RandomEngine *randomEngine) const override;

	virtual bool isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const override;

	virtual bool isSplittable(const FloatType &rootDiameter) const override;

	virtual void serializePartition(std::ofstream &os,
	                                const DiameterEstimator *diameterEstimator,
	                                const DistanceMeasure *distanceMeasure,
	                                RandomEngine *randomEngine) override;

protected:
	mutable FloatType l_;
	mutable FloatType r_;

	mutable std::unique_ptr<std::uniform_real_distribution<FloatType>> sampleDistribution_ = nullptr;

protected:
	VectorFloat sampleUniformlyFromPartition_(RandomEngine *randomEngine) const;

	TreeElement *getSiblingPartition() const;

};

class VDPPartition: public Partition {
public:
	VDPPartition(const std::shared_ptr<Action> &action,
	             TreeElement *const parentPartition,
	             const ProblemEnvironmentOptions *options);

	virtual ~VDPPartition() = default;

	_NO_COPY_BUT_MOVE(VDPPartition)

	const FloatType getL() const;

	const FloatType getR() const;

	virtual Action *sampleAction(RandomEngine *randomEngine) const override;

	virtual std::pair<TreeElement*, TreeElement*> split(const DiameterEstimator *diameterEstimator,
	        const DistanceMeasure *distanceMeasure,
	        RandomEngine *randomEngine) override;

	//virtual void computeDiameter(RandomEngine *randomEngine) const override;

	virtual FloatType getDiameter(const DiameterEstimator *diameterEstimator,
	                              const DistanceMeasure *distanceMeasure,
	                              RandomEngine *randomEngine) const override;

	virtual bool isInPartition(const DistanceMeasure *distanceMeasure, const VectorFloat &action) const override;

	virtual bool isSplittable(const FloatType &rootDiameter) const override;

	virtual void serializePartition(std::ofstream &os,
	                                const DiameterEstimator *diameterEstimator,
	                                const DistanceMeasure *distanceMeasure,
	                                RandomEngine *randomEngine) override;

protected:
	mutable FloatType l = 0.0;
	mutable FloatType r = 2.0 * M_PI;

	//mutable FloatType diam_ = 0.0;

protected:
	VectorFloat sampleUniformlyFromPartition_(RandomEngine *randomEngine) const;

	TreeElement *getSiblingPartition() const;

};
}

#endif