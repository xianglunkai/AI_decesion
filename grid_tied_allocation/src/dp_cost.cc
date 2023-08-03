#include "dp_cost.h"
#include <cmath>
#include <stdio.h>

namespace ai_decision {
namespace grid_tied_allocation {

constexpr float kInf = std::numeric_limits<float>::infinity();


float DpStCost::get_total_cost(const STPoint& point,
			       const float lb,
			       const float ub,
			       const std::vector<std::pair<float, float>>& resonance_intervals,
			       const float policy_reference) const
{

	// check if point is in range
	const float point_cur_allocate = point.s();
	if (point_cur_allocate  < 0 ||
	    point_cur_allocate > ub ||
	    point_cur_allocate < lb ) {
		return kInf;
	}

	// check if point is out side of the resonance intervals
	for (const auto& item : resonance_intervals) {
		auto a = item.first;
		auto b = item.second;

		const float radius = 0.5f * (b - a);
		const float center = 0.5f * (b + a);
		if (radius > std::abs(point_cur_allocate - center)) {
			return kInf;
		}
	}

	// compute the least square
	return (point_cur_allocate - policy_reference) * (point_cur_allocate - policy_reference);
}


} // namespace grid_tied_allo
} // ai_decision
