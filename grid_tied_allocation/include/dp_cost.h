/*
 *  @file: dp_cost.h
 */

#pragma once

#include <vector>
#include <utility>
#include <assert.h>

#include "graph_point.h"
#include "optimizer_config.h"

namespace ai_decision {
namespace grid_tied_allocation {

class DpStCost {
public:
	explicit DpStCost(const OptimizerConfig& config):
	config_(config)
	{
		// todo: check that dimensions agreements
	}


	float get_total_cost(const STPoint& point,
			     const float lb,
			     const float ub,
			     const std::vector<std::pair<float, float>>& resonance_intervals,
			     const float policy_reference) const;

private:
	const OptimizerConfig& config_;
};

} // namespace grid_tied_allo
} // ai_decision
