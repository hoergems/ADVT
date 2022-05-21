#ifndef _OPPT_TREE_GLOBAL_HPP_
#define _OPPT_TREE_GLOBAL_HPP_
#include <functional>

namespace oppt {
class Observation;
typedef std::function<TreeElement*(const Observation *, TreeElement*)> ObservationComparator;
}

#endif