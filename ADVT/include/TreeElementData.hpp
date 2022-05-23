#ifndef _TREE_ELEMENT_DATA_HPP_
#define _TREE_ELEMENT_DATA_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {
class TreeElement;
class TreeElementData {
public:
	_NO_COPY_BUT_MOVE(TreeElementData)
	TreeElementData(TreeElement *const parentElement);

	virtual ~TreeElementData() = default;

	template<class T>
	T* as() {
		return static_cast<T*>(this);
	}

	template<class T>
	T const* as() const
	{
		return static_cast<T const*>(this);
	}

	TreeElement *const getParentElement() const;

	virtual void reset();

protected:
	TreeElement *const parentElement_;

};

typedef std::unique_ptr<TreeElementData> TreeElementDataPtr;
}

#endif