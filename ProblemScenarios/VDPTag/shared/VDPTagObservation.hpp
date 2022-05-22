#ifndef _VDPTAG_OBSERVATION_HPP_
#define _VDPTAG_OBSERVATION_HPP_
#include <oppt/robotHeaders/Observation.hpp>

namespace oppt {
class VDPTagObservation: public VectorObservation {
public:
	using VectorObservation::VectorObservation;

	VDPTagObservation(const VectorFloat &obsVec, const int &beam):
		VectorObservation(obsVec),
		beam_(beam) {

	}

	virtual bool equals(const Observation& otherObservation) const override {
		ERROR("INSIDE EQUALS");
	}

	virtual double distanceTo(const Observation& otherObservation) const override {
		VectorFloat otherObservationVec = otherObservation.as<VectorObservation>()->asVector();
		return math::euclideanDistance(observationVec_, otherObservationVec);
		int commonIdx = 0;
		bool gotIdx = false;
		for (size_t i = 0; i != observationVec_.size(); ++i) {
			bool nan1 = std::isnan(observationVec_[i]);
			bool nan2 = std::isnan(otherObservationVec[i]);
			if ((nan1 == true and nan2 == false) or
			        (nan1 == false and nan2 == true)) {
				return std::numeric_limits<FloatType>::infinity();
			} else if (nan1 == false and nan2 == false) {
				if (gotIdx) {
					printVector(observationVec_, "observationVec_");
					printVector(otherObservationVec, "otherObservationVec");
					cout << "commonIdx: " << commonIdx << endl;
					ERROR("WTF");
				}
				commonIdx = i;
				gotIdx = true;
			}
		}

		return std::fabs(observationVec_[commonIdx] - otherObservationVec[commonIdx]);
	}

	const int beam() const {
		return beam_;
	}

private:
	const int beam_;

};
}

#endif