#pragma once

#include "heuristic_optimizer.h"
#include "nlopt_optimizer.h"

namespace ai_decision {
namespace grid_tied_allocation {

class GridTiedAllocation
{
private:
	const OptimizerConfig& config_;
	GriddedSTGraph dp_;
	NonlinearOptimization nlopt_;
public:
	explicit GridTiedAllocation(const OptimizerConfig& config)
	:dp_(config),nlopt_(config), config_(config){}

	~GridTiedAllocation() = default;

	bool process(const std::vector<float>& current_state,
		     const float reference_command,
		     std::vector<std::pair<uint32_t, float>>* const allocation_result);
};

} // namespace grid_tied_allo
} // ai_decision
