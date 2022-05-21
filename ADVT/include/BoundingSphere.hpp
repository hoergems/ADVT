#ifndef _BOUNDING_SPHERE_HPP_
#define _BOUNDING_SPHERE_HPP_
#include <oppt/opptCore/core.hpp>
#include <CGAL/Cartesian_d.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_d.h>

namespace oppt {
namespace boundingSphere {


typedef CGAL::Cartesian_d<FloatType> K;
typedef std::function<FloatType(const std::vector<VectorFloat> &)> DiamFunction;

struct DiamFunctions {
	template<int DIM>
	static FloatType diam(const std::vector<VectorFloat> &points) {
		typedef CGAL::Min_sphere_of_points_d_traits_d<K, FloatType, DIM>  Traits;
		typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_circle;
		typedef K::Point_d Point;
		Point P[points.size()];
		for (size_t i = 0; i != points.size(); ++i) {
			P[i] = Point(DIM, points[i].begin(), points[i].end());
			//P[i] = Point(points[i][0], points[i][1]);
		}

		Min_circle  mc( P, P + points.size());

		return 2.0 * mc.radius();
	}

	static FloatType diameter(const std::vector<VectorFloat> &points) {
		return (diamFunctionPointers[points[0].size()])(points);		
	}

	static std::vector<DiamFunction> diamFunctionPointers;
};

namespace detail {
template<int DIM>
struct unroller {
	void operator () () {
		DiamFunctions::diamFunctionPointers[DIM] = [](const std::vector<VectorFloat> &points) {
			return DiamFunctions::diam<DIM>(points);
		};		
		unroller<DIM - 1>()();		
	}
};

template<>
struct unroller<0> {
	void operator () () {}
};
}

struct GenerateDiamFunctions {
	static void generate() {
		constexpr int maxDim = 100;
		DiamFunctions::diamFunctionPointers.resize(maxDim + 1);
		detail::unroller<maxDim>()();
	}
};

}
}

#endif