#ifndef _ADVT_OBSERVATION_EDGE_HPP_
#define _ADVT_OBSERVATION_EDGE_HPP_
#include <OpptTree/TreeElement.hpp>
#include <oppt/opptCore/core.hpp>

namespace oppt {
class ObservationEdge: public TreeElement {
public:
	ObservationEdge(TreeElement *const parentElement, const ObservationSharedPtr &observation);

	virtual ~ObservationEdge() = default;

	virtual void print() const override;

	const Observation *getObservation() const;

	template <typename NodeType>
	TreeElement *const getOrCreateChild()
	{
		if (children_.size() == 1)
			return children_[0].get();

		std::unique_ptr<TreeElement> childBelief(new NodeType(this));
		return addChild(std::move(childBelief));
	}

protected:
	const ObservationSharedPtr observation_;	

};
}

#endif