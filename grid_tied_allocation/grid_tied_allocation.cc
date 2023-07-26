#include "grid_tied_allocation.h"

#include <string>
#include <iostream>

namespace ai_decision {
namespace grid_tied_allocation {

bool GridTiedAllocation::process(const std::vector<float>& current_state,
		const float reference_command,
		std::vector<std::pair<uint32_t, float>>* const allocation_result)
{
	// check configuration
	std::string error_message;
	if (!optimizer_config()->check_config(error_message)) {
		std::cout << error_message << std::endl;
		return false;
	}

	// use small load assignment policy
	if (config_.enable_small_load_alg() && small_load_assignment_.process(current_state, reference_command, allocation_result)) {
		// std::cout << " activate small load assignment algorithm" << std::endl;
		return true;
	}

	// using dynamical programming algorithm to generate coarse solution
	std::vector<std::pair<uint32_t, float>> coarse_solution;
	if (!dp_.process(current_state, reference_command, &coarse_solution)) {
		const std::string msg = "DP search failed";
		std::cout << msg << std::endl;
		return false;
	}

	if (!config_.enable_nlopt() || dp_.is_exact_policy()) {
		*allocation_result = std::move(coarse_solution);
		return true;
	}

	// step2: using NLOPT algorithm to find solution exactly
	if (!nlopt_.process(current_state, coarse_solution, reference_command, allocation_result)) {
		const std::string msg = "NLOPT optimization failed";
		std::cout << msg << std::endl;
		return false;
	}

	return true;
}

} // namespace grid_tied_allo
} // ai_decision
