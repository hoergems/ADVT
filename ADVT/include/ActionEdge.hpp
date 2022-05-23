#ifndef _ADVT_ACTION_EDGE_HPP_
#define _ADVT_ACTION_EDGE_HPP_
#include "TreeElement.hpp"
#include "ObservationComparator.hpp"

namespace oppt {
class BeliefNode;
class ActionEdge: public TreeElement {
public:
	friend class BeliefNode;
	ActionEdge(TreeElement *const parentNode,
	           const Action *action);

	virtual ~ActionEdge() = default;

	virtual void print() const override;

	const Action * getAction() const;

	virtual TreeElement *const getOrCreateObservationEdge(const ObservationSharedPtr &observation);

	void setObservationComparator(ObservationComparator observationComparator);	

protected:
	ObservationComparator observationComparator_;

	const Action *action_;	
};

}

#endif