#include "optimizer_config.h"

namespace ai_decision {
namespace grid_tied_allocation {

// singleton instance
OptimizerConfig *OptimizerConfig::singleton_;
OptimizerConfig *optimizer_config()
{
	return OptimizerConfig::get_singleton();
}

// constructor functions
OptimizerConfig::OptimizerConfig()
{
	singleton_ = this;
}

bool OptimizerConfig::check_config(std::string &str_error)
{
	// 1. check allocation type and algorithms
	if (allocation_type_ >= AllocationType::LAST_ALLOCATION) {
		str_error  = std::move("allocator type mismatch");
		return false;
	    }

	if (allocation_algorithm_ >= AllocationAlgorithm::LAST_ALGORITHM) {
		str_error = std::move("allocation algorithm mismatch");
		return false;
	}

	// 2. check allocation resolution
	if (allocation_resolution_ < 1) {
		str_error = std::move("allocation resolution must be greater than 1.0f");
		return false;
	}

	// 3. check each meachine
	uint32_t index_min = 9999;
	float assignment_sum = 0.0f;
	for (size_t i = 0; i < items_config_.size(); ++i) {
		const auto &item = items_config_.at(i);
		if (!item.enabled) {
			continue;
		}
		if (item.lower_bound >= item.upper_bound) {
			str_error  = std::move("lower_bound < upper_bound");
			return false;
		}
		if (item.assigned_factor < 0 || item.assigned_factor > 1) {
			str_error  = std::move("assigned_factor < 0 || > 1");
			return false;
		}
		assignment_sum += item.assigned_factor;
		index_min = (index_min >= i) ? i : index_min;

		for (size_t j = 0; j < item.resonances.size(); j++) {
			const auto &res = item.resonances[j];
			// must be ensure [a, b] is within [lower, upper]
			if (res.first > res.second ||
			    res.first < item.lower_bound ||
			    res.second > item.upper_bound) {
				str_error  = std::move("resonance range error");
				return false;
			}
		}
	}
	if (index_min != 0) {
		str_error  = std::move("index_min > 0");
		return false;
	}

	if (assignment_sum > 1.1) {
		str_error  = std::move("assignment_sum > 1.1");
		return false;
	}
	return true;
}

} // namespace grid_tied_allo
} // ai_decision
