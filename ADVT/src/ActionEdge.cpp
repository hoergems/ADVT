#include "ActionEdge.hpp"
#include "ObservationEdge.hpp"

namespace oppt {
ActionEdge::ActionEdge(TreeElement *const parentNode, const Action *action):
	TreeElement(parentNode),
	action_(action) {

}

void ActionEdge::print() const {

}

const Action * ActionEdge::getAction() const {
	return action_;
}

TreeElement *const ActionEdge::getOrCreateObservationEdge(const ObservationSharedPtr &observation) {
	TreeElement *closestObservationEdge = observationComparator_(observation.get(), this);
	if (closestObservationEdge) {		
		return closestObservationEdge;
	}

	std::unique_ptr<TreeElement> observationEdge(new ObservationEdge(this, observation));
	auto observationEdgeAdd = addChild(std::move(observationEdge));
	return observationEdgeAdd;
}

void ActionEdge::setObservationComparator(ObservationComparator observationComparator) {
	observationComparator_ = observationComparator;
}
}