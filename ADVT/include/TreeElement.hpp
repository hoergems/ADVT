#ifndef _TREE_ELEMENT_HPP_
#define _TREE_ELEMENT_HPP_
#include "TreeElementData.hpp"
#include <oppt/opptCore/core.hpp>

namespace oppt {
/** Forward declaration of oppt::Tree */
class Tree;
/** Forward declaration of oppt::Tree */
class TreeElement;
/** @brief A unique_ptr to a oppt::TreeElement */
typedef std::unique_ptr<TreeElement> TreeElementPtr;
/** @brief A vector of unique_ptr to a oppt::TreeElement */
typedef std::vector<std::unique_ptr<TreeElement>> VectorTreeElement;
/** @brief A iterator over the elements of oppt::VectorTreeElement */
typedef VectorTreeElement::iterator ChildrenIterator;

class TreeElement {
public:	
	friend class Tree;
	_NO_COPY_BUT_MOVE(TreeElement)
	TreeElement(TreeElement *const parentElement);

	virtual ~TreeElement() = default;

	/**
	 * @brief Static cast to type T
	 * @tparam T The action type to cast to
	 */
	template<class T>
	T* as() {
		return static_cast<T*>(this);
	}

	template<class T>
	T const* as() const
	{
		return static_cast<T const*>(this);
	}

	/**
	 * @brief Prints the oppt::TreeElement
	 */
	virtual void print() const = 0;

	/**
	 * @brief Get the ID 
	 */
	long getId() const;

	/**
	 * @brief Get a pointer to the parent element. if no such element exists, a nullptr is returned
	 */
	TreeElement *const getParent() const;

	/**
	 * @brief Get a pointer to the tree this oppt::TreeElement belongs to
	 */
	Tree *const getTree() const;

	/**
	 * @brief Get a oppt::ChildrenIterator to the child elements
	 */
	ChildrenIterator getChildren();

	/**
	 * @brief Removes ownership of a child oppt::TreeElement and returns a unique pointer to it
	 */
	TreeElementPtr releaseChild(TreeElement *const child);

	/**
	 * @brief Remove all subtrees by deleting this oppt::TreeElement s children
	 */
	void removeSubtrees();

	/**
	 * @brief Get the number of children
	 */
	size_t getNumChildren() const;

	/**
	 * @brief Add a oppt::TreeElement to the set of children
	 */
	TreeElement* addChild(TreeElementPtr childElement);

	/**
	 * @brief Removes a child
	 */
	bool removeChild(const TreeElement *child);

	/**
	 * @brief Set the oppt::TreeElementData
	 */
	void setData(std::unique_ptr<TreeElementData> treeElementData);

	/**
	 * @brief Get a pointer to the oppt::TreeElementData
	 */
	TreeElementData *const getData() const;

	/**
	 * @brief Returns true if this TreeElement is the root
	 */
	bool isRoot() const;

	virtual void reset();

protected:
	long id_;

	TreeElement *parentElement_;

	VectorTreeElement children_;

	std::unique_ptr<TreeElementData> treeElementData_;

	Tree *tree_ = nullptr;
};


}

#endif