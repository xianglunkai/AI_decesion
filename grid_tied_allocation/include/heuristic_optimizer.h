/*
 *  @file: heuristic_optimizer.h
 */
#pragma once

#include "dp_cost.h"
#include <memory>

namespace ai_decision {
namespace grid_tied_allocation {

class GriddedSTGraph {
public:
	explicit GriddedSTGraph(const OptimizerConfig& config);

	/* Do not allow copies */
	GriddedSTGraph(const GriddedSTGraph &other) = delete;
	GriddedSTGraph &operator=(const GriddedSTGraph) = delete;

	bool process(const std::vector<float>& current_state,
		     const float reference_allocation_command,
		     std::vector<std::pair<uint32_t, float>>* const allocation_result) WARN_IF_UNUSED;

	const bool is_exact_policy() const WARN_IF_UNUSED { return is_exact_policy_; }

private:
	bool init_items_config(const float reference_allocation_command);

	bool init_cost_table();

	bool init_reference_allocation_lookup(const std::vector<float>& current_state, const float reference_allocation_command);

	bool check_policy_output(std::vector<std::pair<uint32_t, float>>* const allocation_result);

	bool retrieve_allocation_profile(std::vector<std::pair<uint32_t, float>>* const allocation_result);

	bool calculate_total_cost();

	// defined for cyber task
	struct StGraphMessage {
		StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
		uint32_t c;
		uint32_t r;
	};

	void calculate_cost_at(const std::shared_ptr<StGraphMessage>& msg);

private:
	// cost utility with configuration;
	DpStCost dp_st_cost_;

	uint32_t total_length_t_{0};
	uint32_t unit_t_{1};
	uint32_t dimension_t_{0};

	float total_length_s_{0.0f};
	float unit_s_ = 0.0;
	uint32_t dimension_s_ = 0;

	// policy requirements
	AllocationType allocation_type_;
	std::vector<std::pair<uint32_t, float>> policy_reference_;
	bool is_exact_policy_{false};

	// dp st configuration
	const OptimizerConfig& gridded_graph_config_;
	std::vector<OptimizerConfig::ItemConfig> items_config_;

	// cost_table_[t][s]
	// row: s, col: t --- NOTICE: Please do NOT change.
	std::vector<std::vector<STGraphPoint>> cost_table_;
};

} // namespace grid_tied_allo
} // ai_decision
