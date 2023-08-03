#include <algorithm>
#include <limits>
#include <string>
#include <iostream>
#include <cmath>
#include <mutex>

#include "heuristic_optimizer.h"

namespace ai_decision {
namespace grid_tied_allocation {

GriddedSTGraph::GriddedSTGraph(const OptimizerConfig& config)
 : gridded_graph_config_(config),
   dp_st_cost_(config)
{}


bool GriddedSTGraph::process(const std::vector<float>& current_state,
			     const float reference_allocation_command,
		             std::vector<std::pair<uint32_t, float>>* const allocation_result)
{
	if (!init_items_config(reference_allocation_command)) {
		const std::string msg = "Initialize items config failed.";
		std::cout << msg << std::endl;
		return false;
	}

	if (!init_reference_allocation_lookup(current_state, reference_allocation_command)) {
	 	const std::string msg = "Initialize refernce allocation lookup table failed.";
		std::cout << msg << std::endl;
		return false;
	}

	if (check_policy_output(allocation_result)) {
		return true;
	}

	if (!init_cost_table()) {
		const std::string msg = "Initialize cost table failed.";
		std::cout << msg << std::endl;
		return false;
	}

	if (!calculate_total_cost()) {
		const std::string msg = "Calculate total cost failed.";
		std::cout << msg << std::endl;
		return false;
	}

	if (!retrieve_allocation_profile(allocation_result)) {
		const std::string msg = "Retrieve best allocation profile failed.";
		std::cout << msg << std::endl;
		return false;
	}
	return true;
}

bool GriddedSTGraph::init_items_config(const float reference_allocation_command)
{
	// collect all items that is enabled
	items_config_.clear();
	auto& params = gridded_graph_config_.items_config();
	for (auto& item : params) {
		if (item.enabled) {
			items_config_.emplace_back(item);
		}
	}

	total_length_t_ = items_config_.size();
	unit_t_ = 1;

	total_length_s_ = reference_allocation_command;
	unit_s_ = gridded_graph_config_.allocation_resolution();

	return total_length_s_ >= 0 && !items_config_.empty();
}

bool GriddedSTGraph::init_cost_table()
{
	dimension_t_ = static_cast<uint32_t>(std::ceil(total_length_t_ /  unit_t_));
	dimension_s_ = static_cast<uint32_t>(std::ceil(total_length_s_ / unit_s_)) + 1;

	if (dimension_t_ < 1) {
		const std::string msg = "DP cost table size incorrect";
		std::cout << msg << std::endl;
		return false;
	}

	cost_table_ = std::vector<std::vector<STGraphPoint>>(
		dimension_t_, std::vector<STGraphPoint>(dimension_s_, STGraphPoint()));

	// for convienience ,we begin planning  from 0
	uint32_t curr_t = 0;
	for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
		auto& cost_table_i = cost_table_.at(i);
		float curr_s = 0.0f;
		for (uint32_t j = 0; j < cost_table_i.size(); ++j, curr_s += unit_s_) {
			cost_table_i[j].init(i, j, STPoint(curr_s, curr_t));
		}
	}

	return true;

}

bool GriddedSTGraph::init_reference_allocation_lookup(const std::vector<float>& current_state, const float reference_allocation_command)
{
	allocation_type_ = gridded_graph_config_.allocation_type();

	// update current allocation states
	policy_reference_.clear();
	policy_reference_.reserve(total_length_t_);

	switch (allocation_type_)
	{
	case AllocationType::PROPORTIONAL_ALLOCATION:
		{
			for (size_t i = 0; i < total_length_t_; i++) {
				const float u = items_config_.at(i).assigned_factor * reference_allocation_command;
				const uint32_t index = items_config_.at(i).index;
				policy_reference_.emplace_back(std::make_pair(index, u));
			}
		}
		break;
	case AllocationType::MARGIN_ALLOCATION:
		{
			std::vector<std::pair<float, float>> xv_pairs;
			xv_pairs.reserve(total_length_t_);
			for (size_t i = 0; i < total_length_t_; i++) {
				const uint32_t index = items_config_.at(i).index;
				std::pair<float, float> it = std::make_pair(current_state[index],
							 items_config_.at(i).upper_bound - items_config_.at(i).lower_bound);
				xv_pairs.emplace_back(it);
			}

			float sum_x = 0.0f;
			float sum_v_x = 0.0f;
			for (auto& xv: xv_pairs) {
				sum_x += xv.first;
				sum_v_x += (xv.second - xv.first);
			}


			const float allocation = reference_allocation_command - sum_x;
			for (size_t i = 0; i < total_length_t_; i++) {
				const uint32_t index = items_config_.at(i).index;
				float allocation_delta = 0;
				if (reference_allocation_command > sum_x) {
					allocation_delta = allocation * (xv_pairs[i].second - xv_pairs[i].first) / sum_v_x;
				} else {
					allocation_delta = allocation * xv_pairs[i].first / sum_x;
				}
				policy_reference_.emplace_back(std::make_pair(index, allocation_delta + xv_pairs[i].first));
			}
		}
		break;
	default:
		break;
	}

	return true;
}


bool GriddedSTGraph::calculate_total_cost()
{
	// col and row are for STGraph
	// t corresponding to col
	// s corresponding to row
	for (size_t c = 0; c < cost_table_.size(); c++) {
		auto& cost_table_i = cost_table_.at(c);
		const size_t next_highest_row = cost_table_i.size();
		std::vector<std::future<void>> results;

		for (size_t r = 0; r < next_highest_row; r++) {
			auto msg = std::make_shared<StGraphMessage>(c, r);

			if (gridded_graph_config_.enable_multi_threads_in_dp()) {
				results.push_back(Async(&GriddedSTGraph::calculate_cost_at, this, msg));
			} else {
				calculate_cost_at(msg);
			}
		}

		if (gridded_graph_config_.enable_multi_threads_in_dp()) {
			for (auto& result : results) {
				result.get();
			}
		}
	}
	return true;
}


bool GriddedSTGraph::check_policy_output(std::vector<std::pair<uint32_t, float>>* const allocation_result)
{
	is_exact_policy_ = false;
	std::vector<std::pair<uint32_t, float>> result;
	for (size_t i = 0; i  < policy_reference_.size(); ++i) {
		const uint32_t index = items_config_.at(i).index;
		float value = policy_reference_.at(i).second;

		// check value is within range
		if (value > items_config_.at(i).upper_bound || value < items_config_.at(i).lower_bound) {
			return false;
		}

		// check value is within resonance
		const auto& resonances = items_config_.at(i).resonances;
		for (const auto& zone : resonances) {
			const auto& a = zone.first;
			const auto& b = zone.second;
			if (value >a && value <b) {
				return false;
			}
		}

		result.emplace_back(std::make_pair(index, value));
	}

	*allocation_result = std::move(result);
	is_exact_policy_ = true;

	return true;
}

bool GriddedSTGraph::retrieve_allocation_profile(std::vector<std::pair<uint32_t, float>>* allocation_result)
{
	const STGraphPoint* best_end_point = nullptr;
	const std::vector<STGraphPoint> &end_points = cost_table_.back();
	for (auto it = end_points.rbegin(); it != end_points.rend(); ++it) {
		if (!std::isinf(it->total_cost())) {
			best_end_point = &(*it);
			break;
		}
	}

	if (best_end_point == nullptr) {
		const std::string msg = "Fail to find the best feasible trajectory.";
		std::cout << msg << std::endl;
		return false;
	}

	std::vector<STPoint> allocation_profile;
	const STGraphPoint* cur_point = best_end_point;
	while (cur_point != nullptr) {
		allocation_profile.push_back(cur_point->point());
		cur_point = cur_point->pre_point();
	}
  	std::reverse(allocation_profile.begin(), allocation_profile.end());

	std::vector<std::pair<uint32_t, float>> result;
	for (size_t i = 0; i  < allocation_profile.size(); ++i) {
		const uint32_t index = items_config_.at(i).index;
		float value = 0;
		if (i == 0) {
			value = allocation_profile.at(i).s();
		} else {
			value = allocation_profile.at(i).s() - allocation_profile.at(i - 1).s();
		}
		result.emplace_back(std::make_pair(index, value));
	}

	*allocation_result = std::move(result);

	return true;
}


void GriddedSTGraph::calculate_cost_at(const std::shared_ptr<StGraphMessage>& msg)
{
	const uint32_t c = msg->c;
	const uint32_t r = msg->r;
	auto& cost_cr = cost_table_[c][r];
	auto& point = cost_cr.point();

	if (c == 0) {
		cost_cr.set_total_cost(
			dp_st_cost_.get_total_cost(
				point,
				items_config_.at(c).lower_bound,
				items_config_.at(c).upper_bound,
				items_config_.at(c).resonances,
				policy_reference_.at(c).second));
		return;
	}

 	const auto& pre_col = cost_table_[c - 1];
	for (size_t i = 0; i <= r; i++) {
		if (std::isinf(pre_col[i].total_cost())) {
			continue;
		}

		const float cost = pre_col[i].total_cost() +
				   dp_st_cost_.get_total_cost(
					STPoint((r - i) * unit_s_, c),
					items_config_.at(c).lower_bound,
					items_config_.at(c).upper_bound,
					items_config_.at(c).resonances,
					policy_reference_.at(c).second);

		if (cost < cost_cr.total_cost()) {
			cost_cr.set_total_cost(cost);
			cost_cr.set_pre_point(pre_col[i]);
		}
	}

}
} // namespace grid_tied_allo
} // ai_decision
