#ifndef _ADVT_FILTER_HPP_
#define _ADVT_FILTER_HPP_
#include <oppt/filter/particleFilter/ParticleFilter.hpp>

namespace oppt {
class ADVTFilter: public ParticleFilter {
public:
	ADVTFilter();

	virtual ~ADVTFilter() = default;

	virtual FilterResultPtr propagateParticles(const FilterRequestPtr& filterRequest) override;

};

}

#endif