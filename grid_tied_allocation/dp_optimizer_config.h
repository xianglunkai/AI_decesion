/*
 *  @file: dp_optimizer_config.h
 */
#pragma once

#include <limits>
#include <cstdint>
#include <vector>
#include <utility>
#include <cstdint>

#include "common.h"

namespace ai_decision {
namespace grid_tied_allocation {

enum class AllocationType: std::uint8_t {
	PROPORTIONAL_ALLOCATION = 0,
	MARGIN_ALLOCATION = 1
};

class DpOptimizerConfig {
public:
	DpOptimizerConfig();

	CLASS_NO_COPY(DpOptimizerConfig);  /* Do not allow copies */

	// get singleton instance
	static DpOptimizerConfig *get_singleton() {
		return singleton_;
	}

	struct ItemConfig {
		uint32_t index;
		bool enabled;
		float lower_bound;
		float upper_bound;
		float assigned_factor;
		std::vector<std::pair<float, float>> resonances;
	};

	// rest configuration
	void clear_config() { items_config_.clear(); }

	// write configuration parameters
	void items_config(std::vector<ItemConfig>& config) { items_config_ = config; }

	void allocation_type(const AllocationType& allocation_type) { allocation_type_ = allocation_type; }

	void allocation_resolution(const float allocation_resolution) { allocation_resolution_ = allocation_resolution; }

	// read configuration parameters
	const std::vector<ItemConfig>& items_config() const { return items_config_; }

	const AllocationType allocation_type() const { return allocation_type_; }

	const float allocation_resolution() const { return allocation_resolution_;}

private:
  	static DpOptimizerConfig *singleton_;
	std::vector<ItemConfig>  items_config_;
	AllocationType allocation_type_{AllocationType::MARGIN_ALLOCATION};
	float allocation_resolution_{10.0f};
};

} // namespace grid_tied_allo
} // ai_decision
