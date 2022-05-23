#include "TreeElementData.hpp"
#include "TreeElement.hpp"

namespace oppt {
TreeElementData::TreeElementData(TreeElement *const parentElement):
	parentElement_(parentElement) {

}

TreeElement *const TreeElementData::getParentElement() const {
	return parentElement_;
}

void TreeElementData::reset() {

}


}