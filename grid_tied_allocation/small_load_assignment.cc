#include "small_load_assignment.h"
#include <algorithm>
#include <limits>
#include <string>
#include <iostream>
#include <cmath>
#include <assert.h>
#include <tuple>

namespace ai_decision {
namespace grid_tied_allocation {

bool SmallLoadAssignment::process(const std::vector<float>& current_state,
		     const float reference_allocation_command,
		     std::vector<std::pair<uint32_t, float>>* const allocation_result)
{
	if (!init_items_config()) {
		const std::string msg = "Initialize items config failed for SmallLoadAssignment.";
		std::cout << msg << std::endl;
		return false;
	}

	// create solution structure
	std::vector<std::pair<uint32_t, float>> solution;
	create_solution_vector(current_state, solution);

	// small load assignment check
	float assignment_delta = reference_allocation_command - sum_of_solution(solution);

	const float dead_zone = gridded_graph_config_.small_load_dead_size();
	// std::cout << std::abs(assignment_delta) << "-" << dead_zone << std::endl;
	if (std::abs(assignment_delta)  > dead_zone ) {
		return false;
	}

	if (!retrieve_allocation_profile(assignment_delta, solution, allocation_result)) {
		return false;
	}


	return true;
}

bool SmallLoadAssignment::retrieve_allocation_profile(float &assignment_delta,  std::vector<std::pair<uint32_t, float>> &solution, std::vector<std::pair<uint32_t, float>>* const allocation_result)
{
	bool assig_success = false;
	if (assignment_delta >= 0) {
		std::sort(solution.begin(), solution.end(),
		 [this](std::pair<uint32_t, float> a, std::pair<uint32_t, float> b) {
			const uint32_t index_a = a.first;
			const uint32_t index_b = b.first;
			const float vol_a = gridded_graph_config_.items_config().at(index_a).upper_bound - gridded_graph_config_.items_config().at(index_a).lower_bound;
			const float vol_b = gridded_graph_config_.items_config().at(index_b).upper_bound - gridded_graph_config_.items_config().at(index_b).lower_bound;
			return (vol_a - a.second) >= (vol_b - b.second);
		 });


		for (auto & sol : solution) {
			const float temp = sol.second + assignment_delta;
			const uint32_t index = sol.first;

			const auto & res = gridded_graph_config_.items_config().at(index).resonances;
			float lower = gridded_graph_config_.items_config().at(index).lower_bound;
			float upper = gridded_graph_config_.items_config().at(index).upper_bound;

			if (!find_vibration_zone(temp, res, lower, upper)) {
				sol.second = temp;
				assig_success = true;
				break;
			}


			assignment_delta -= (lower - sol.second);
			sol.second = lower;
		}


	} else {
		std::sort(solution.begin(), solution.end(),
		 [this](std::pair<uint32_t, float> a, std::pair<uint32_t, float> b) {
			const uint32_t index_a = a.first;
			const uint32_t index_b = b.first;
			const float vol_a = gridded_graph_config_.items_config().at(index_a).upper_bound - gridded_graph_config_.items_config().at(index_a).lower_bound;
			const float vol_b = gridded_graph_config_.items_config().at(index_b).upper_bound - gridded_graph_config_.items_config().at(index_b).lower_bound;
			return (vol_a - a.second) <= (vol_b - b.second);
		 });

		for (auto & sol : solution) {
			const float temp = sol.second + assignment_delta;
			const uint32_t index = sol.first;

			const auto & res = gridded_graph_config_.items_config().at(index).resonances;
			float lower = gridded_graph_config_.items_config().at(index).lower_bound;
			float upper = gridded_graph_config_.items_config().at(index).upper_bound;

			if (!find_vibration_zone(temp, res, lower, upper)) {
				sol.second = temp;
				assig_success = true;
				break;
			}

			assignment_delta -= (upper - sol.second);
			sol.second = upper;
		}
	}

	if (assig_success) {
		std::sort(solution.begin(), solution.end(),
			[](std::pair<uint32_t, float> a, std::pair<uint32_t, float> b) {
			const uint32_t index_a = a.first;
			const uint32_t index_b = b.first;
			return index_a <= index_b;
			});
		*allocation_result = std::move(solution);
	}

	return assig_success;
}


bool SmallLoadAssignment::init_items_config(void)
{
	// collect all items that is enabled
	items_config_.clear();
	auto& params = gridded_graph_config_.items_config();
	for (auto& item : params) {
		if (item.enabled) {
			items_config_.emplace_back(item);
		}
	}

	// update optimization dimensions
	dimension_ = items_config_.size();

	// return true if items_config is not empty
	return !items_config_.empty();
}

bool SmallLoadAssignment::find_vibration_zone(const float x, const std::vector<std::pair<float, float>> &resonances, float &lower, float &upper)
{
	bool in_vibration_zone = false;

	if ( x < lower ) {
		upper = lower;
		lower = x;
		in_vibration_zone = true;
	} else if ( x > upper ) {
		lower =  upper;
		upper =  x;
		in_vibration_zone = true;
	} else {
		for (const auto & res: resonances ) {
			if (x >= res.first && x <= res.second) {
				lower = res.first;
				upper = res.second;
				in_vibration_zone = true;
				break;
			}
		}
	}

	return in_vibration_zone;

}


void SmallLoadAssignment::create_solution_vector(const std::vector<float>& current_state, std::vector<std::pair<uint32_t, float>> &solution)
{
	solution.clear();
	solution.reserve(dimension_);
	for (const auto& item : items_config_) {
		const auto index = item.index;
		solution.emplace_back(std::make_pair(index, current_state.at(index)));

	}
}

float SmallLoadAssignment::sum_of_solution(const std::vector<std::pair<uint32_t, float>> &solution) const
{
	float sum = 0;
	for (const auto & item : solution) {
		sum += item.second;
	}

	return sum;
}

} // namespace grid_tied_allo
} // ai_decision
