#pragma once

#include <utility>
#include <vector>
#include <cstdint>


#include "optimizer_config.h"

namespace ai_decision {
namespace grid_tied_allocation {

class NonlinearOptimization
{
private:
	// policy requirements
	AllocationType allocation_type_;
	std::vector<float> policy_reference_;

	// configuration parameters
	const OptimizerConfig& original_config_;
	std::vector<OptimizerConfig::ItemConfig> items_config_;

	// optimization parameters
	std::size_t opt_dimension_;
	std::vector<double> opt_lb_;
	std::vector<double> opt_ub_;

public:
	explicit NonlinearOptimization(const OptimizerConfig& config);
	~NonlinearOptimization() = default;

	bool process(const std::vector<float>& current_state,
		     const std::vector<std::pair<uint32_t, float>>& guess_solution,
		     const float reference_command,
		     std::vector<std::pair<uint32_t, float>>* const solution);

private:
	bool init_items_config(const float reference_command);

	bool retrieve_allocation_profile(const float reference_command, const std::vector<std::pair<uint32_t, float>>& guess_solution, std::vector<std::pair<uint32_t, float>>* const solution);

	bool init_reference_allocation_lookup(const std::vector<float>& current_state, const float reference_allocation_command);

	static double objective(const std::vector<double>& x, std::vector<double>& grad, void* data);

	static double inequality_constraint(const std::vector<double>& x, std::vector<double>& grad, void* data);

	static double equality_constraint(const std::vector<double>& x, std::vector<double>& grad, void* data);
};

} // namespace grid_tied_allo
} // ai_decision
