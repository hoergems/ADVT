#include "Partition.hpp"
#include "PartitionNode.hpp"
#include <oppt/global.hpp>

namespace oppt {
Partition::Partition(TreeElement *const parentPartition,
                     const std::shared_ptr<Action> &action,
                     const ProblemEnvironmentOptions *options):
	TreeElement(parentPartition),
	action_(action),
	options_(options),
	id_(oppt::UID::getUniqueId()) {

}

size_t Partition::getID() const {
	return id_;
}


void Partition::print() const {

}

}