#include "Tree.hpp"
#include "TreeElement.hpp"

namespace oppt {
TreeElement *Tree::getRoot() const {
	if (!root_)
		ERROR("No root");
	return root_.get();
}

void Tree::updateRoot(std::unique_ptr<TreeElement> root) {
	root_ = std::move(root);
	root_->parentElement_ = nullptr;
	root_->tree_ = this;
}

void Tree::print() const {

}
}