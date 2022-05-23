#include "TreeElement.hpp"
#include "TreeElementData.hpp"
#include "Tree.hpp"
#include <oppt/opptCore/UID.hpp>

namespace oppt {
TreeElement::TreeElement(TreeElement *const parentElement):
	parentElement_(parentElement),
	id_(oppt::UID::getUniqueId()) {
	if (parentElement)
		tree_ = parentElement->getTree();
	
}

long TreeElement::getId() const {
	return id_;
}


TreeElement *const TreeElement::getParent() const {
	return parentElement_;
}

Tree *const TreeElement::getTree() const {
	return tree_;
}

TreeElement* TreeElement::addChild(std::unique_ptr<TreeElement> childElement) {
	childElement->tree_ = tree_;
	children_.push_back(std::move(childElement));
	return children_[children_.size() - 1].get();
}

ChildrenIterator TreeElement::getChildren() {
	return children_.begin();
}

TreeElementPtr TreeElement::releaseChild(TreeElement *const child) {
	size_t idx = 0;
	for (auto &ch : children_) {
		if (ch.get() == child) {
			TreeElementPtr uniqueChild = std::move(ch);
			children_[idx] = nullptr;
			children_.erase(children_.begin() + idx);
			return std::move(uniqueChild);
		}

		idx++;
	}

	return nullptr;
}

void TreeElement::removeSubtrees() {
	children_.clear();
}

size_t TreeElement::getNumChildren() const {
	return children_.size();
}


bool TreeElement::removeChild(const TreeElement *child) {
	for (auto childIterator = getChildren(); childIterator != children_.end(); childIterator++) {
		if ((*childIterator).get() == child) {
			childIterator = children_.erase(childIterator);
			return true;
		}
	}

	return false;
}

void TreeElement::reset() {
	if (treeElementData_)
		treeElementData_->reset();
	for (auto &child : children_) {
		child->reset();
	}
}


void TreeElement::setData(std::unique_ptr<TreeElementData> treeElementData) {
	treeElementData_ = std::move(treeElementData);
}


TreeElementData *const TreeElement::getData() const {
	return treeElementData_.get();
}

bool TreeElement::isRoot() const {
	if (!tree_)
		ERROR("Tree is null");
		
	return tree_->getRoot() == this;
}

}