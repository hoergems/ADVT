#ifndef _OPPT_TREE_HPP_
#define _OPPT_TREE_HPP_
#include <oppt/opptCore/core.hpp>
#include "TreeElement.hpp"

namespace oppt {
/** @brief Forward declaration of oppt::TreeElement */
//class TreeElement;

class Tree {
public:
	_NO_COPY_BUT_MOVE(Tree)
	
	Tree() = default;

	virtual ~Tree() = default;

	template<class NodeType>
	void initRoot() {
		root_ = std::unique_ptr<NodeType>(new NodeType(nullptr));
		root_->tree_ = this;
	}

	TreeElement *getRoot() const;

	void updateRoot(std::unique_ptr<TreeElement> root);

	virtual void print() const;

private:
	std::unique_ptr<TreeElement> root_;

};
}

#endif