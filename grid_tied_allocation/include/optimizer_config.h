/*
 *  @file: dp_optimizer_config.h
 */
#pragma once

#include <limits>
#include <cstdint>
#include <vector>
#include <utility>
#include <cstdint>
#include <string>

#include "common.h"

namespace ai_decision {
namespace grid_tied_allocation {

enum class AllocationType: std::uint8_t {
	PROPORTIONAL_ALLOCATION = 0,
	MARGIN_ALLOCATION = 1,
	LAST_ALLOCATION = 2
};

enum class AllocationAlgorithm: std::uint8_t {
	NONE = 0,
	OSQP = 1,
	NLOPT = 2,
	LAST_ALGORITHM = 3
};

class OptimizerConfig {
public:
	OptimizerConfig();

	CLASS_NO_COPY(OptimizerConfig);  /* Do not allow copies */

	// get singleton instance
	static OptimizerConfig *get_singleton() {
		return singleton_;
	}

	struct ItemConfig {
		uint32_t index; 	 // meachine index
		bool enabled;    	 // true if meachine could be allocated
		float lower_bound;	 // meachine minimal capacity
		float upper_bound;       // meachine maximal capacity
		float assigned_factor;   // default assigned factor
		std::vector<std::pair<float, float>> resonances;  // resonances zones
	};

	// rest configuration
	void clear_config() { items_config_.clear(); }

	bool check_config(std::string &str_error);

	// write configuration parameters
	void items_config(std::vector<ItemConfig>& config) { items_config_ = config; }

	void allocation_type(const AllocationType& allocation_type) { allocation_type_ = allocation_type; }

	void allocation_resolution(const float allocation_resolution) { allocation_resolution_ = allocation_resolution; }

	void allocation_algorithm(const AllocationAlgorithm& allocation_algorithm) { allocation_algorithm_ = allocation_algorithm; }

	void enable_multi_threads_in_dp(const bool enable_multi_threads_in_dp) { enable_multi_threads_in_dp_ = enable_multi_threads_in_dp; }

	void small_load_dead_size(const float small_load_dead_size) { small_load_dead_size_ = small_load_dead_size;}

	void enable_small_load_alg(const bool enable_small_load_alg) { enable_small_load_alg_ = enable_small_load_alg; }

	// read configuration parameters
	const std::vector<ItemConfig>& items_config() const { return items_config_; }

	const AllocationType allocation_type() const { return allocation_type_; }

	const float allocation_resolution() const { return allocation_resolution_;}

	const AllocationAlgorithm allocation_algorithm() const { return allocation_algorithm_; }

	const bool enable_multi_threads_in_dp() const { return enable_multi_threads_in_dp_; }

	const float small_load_dead_size() const { return small_load_dead_size_; }

	const bool enable_small_load_alg() const { return enable_small_load_alg_; }

private:
  	static OptimizerConfig *singleton_;

	std::vector<ItemConfig>  items_config_;

	AllocationType allocation_type_{AllocationType::MARGIN_ALLOCATION};

	float allocation_resolution_{10.0f};

	bool enable_multi_threads_in_dp_{false};

	bool enable_small_load_alg_{false};

	float small_load_dead_size_{100.0f};

	AllocationAlgorithm allocation_algorithm_{AllocationAlgorithm::NONE};
};

OptimizerConfig *optimizer_config();

} // namespace grid_tied_allo
} // ai_decision
