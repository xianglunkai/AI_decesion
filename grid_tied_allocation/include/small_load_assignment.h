/*
 *  @file: small_load_assigment.h
 */
#pragma once

#include "optimizer_config.h"

#include <memory>

namespace ai_decision {
namespace grid_tied_allocation {

class SmallLoadAssignment {
public:
	explicit SmallLoadAssignment(const OptimizerConfig& config)
	: gridded_graph_config_(config)
	{}

	/* Do not allow copies */
	CLASS_NO_COPY(SmallLoadAssignment);


	bool process(const std::vector<float>& current_state,
		     const float reference_allocation_command,
		     std::vector<std::pair<uint32_t, float>>* const allocation_result) WARN_IF_UNUSED;

private:
	const OptimizerConfig& gridded_graph_config_;
	std::vector<OptimizerConfig::ItemConfig> items_config_;
	std::size_t dimension_;

private:
	bool init_items_config(void);

	bool retrieve_allocation_profile(float &assignment_delta,  std::vector<std::pair<uint32_t, float>> &solution, std::vector<std::pair<uint32_t, float>>* const allocation_result) WARN_IF_UNUSED;

	bool find_vibration_zone(const float x, const std::vector<std::pair<float, float>> &resonances, float &lower, float &upper);

	void create_solution_vector(const std::vector<float>& current_state, std::vector<std::pair<uint32_t, float>> &solution);

	float sum_of_solution(const std::vector<std::pair<uint32_t, float>> &solution) const;
};

} // namespace grid_tied_allo
} // ai_decision
