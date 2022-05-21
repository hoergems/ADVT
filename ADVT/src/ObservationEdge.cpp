#include "ObservationEdge.hpp"

namespace oppt {
ObservationEdge::ObservationEdge(TreeElement *const parentElement, const ObservationSharedPtr &observation):
	TreeElement(parentElement),
	observation_(observation) {

}

void ObservationEdge::print() const {

}

const Observation *ObservationEdge::getObservation() const {
	return observation_.get();
}
}