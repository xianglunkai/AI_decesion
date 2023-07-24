#include <nlopt.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <iostream>
#include <cmath>
#include <assert.h>
#include <tuple>

#include "nlopt_optimizer.h"


namespace ai_decision {
namespace grid_tied_allocation {

NonlinearOptimization::NonlinearOptimization(const OptimizerConfig& config)
	: original_config_(config)
{}

bool NonlinearOptimization::process(const std::vector<float>& current_state,
		const std::vector<std::pair<uint32_t, float>>& guess_solution,
		const float reference_command,
		std::vector<std::pair<uint32_t, float>>* const solution)
{
	if (!init_items_config(reference_command)) {
		const std::string msg = "Initialize items config failed for NLOPT.";
		std::cout << msg << std::endl;
		return false;
	}

	if (!init_reference_allocation_lookup(current_state, reference_command)) {
	 	const std::string msg = "Initialize refernce allocation lookup table failed for NLOPT.";
		std::cout << msg << std::endl;
		return false;
	}

	if (!retrieve_allocation_profile(reference_command, guess_solution, solution)) {
		const std::string msg = "Retrieve best allocation profile failed for NLOPT.";
		std::cout << msg << std::endl;
		return false;
	}
	return true;
}

bool NonlinearOptimization::init_items_config(const float reference_command)
{
	// collect all items that is enabled
	items_config_.clear();
	auto& params = original_config_.items_config();
	for (auto& item : params) {
		if (item.enabled) {
			items_config_.emplace_back(item);
		}
	}

	// update lb and ub
	opt_lb_.clear(); opt_ub_.clear();
	opt_lb_.reserve(items_config_.size());
	opt_ub_.reserve(items_config_.size());
	for (auto& item : items_config_) {
		opt_lb_.push_back(item.lower_bound);
		opt_ub_.push_back(item.upper_bound);
	}

	// update optimization dimensions
	opt_dimension_ = items_config_.size();

	// return true if items_config is not empty
	return !items_config_.empty();
}


void NonlinearOptimization::build_bounds(const std::vector<double> &x, std::vector<double> &lb, std::vector<double> &ub)
{
	for (size_t i = 0; i < opt_dimension_; i++) {
		if (is_negative(x[i] - lb[i]) || is_zero(x[i] - lb[i])) {
			ub[i] = lb[i];
			continue;
		}

		double lower = lb[i];
		bool find_bound = false;
		const auto& item = items_config_.at(i).resonances;
		for (const auto& constraint : item) {
			const auto a = constraint.first;
			const auto b = constraint.second;
			if (is_negative(x[i] - a) || is_zero(x[i] - a)) {
				lb[i] = lower;
				ub[i] = a;
				find_bound = true;
				break;
			}

			lower = b;
		}

		if (!find_bound) {
			lb[i] = lower;
		}
	}

}

bool NonlinearOptimization::retrieve_allocation_profile(const float reference_command,
							const std::vector<std::pair<uint32_t, float>>& guess_solution,
							std::vector<std::pair<uint32_t, float>>* const solution)
{
	assert(guess_solution.size() == opt_dimension_);

	// check guess solution index
	std::vector<double> x;
	x.reserve(opt_dimension_);
	for (size_t i = 0; i < opt_dimension_; ++i) {
		if (items_config_[i].enabled && guess_solution[i].first == items_config_[i].index) {
			x.push_back(guess_solution[i].second);
		}
	}

	// try to check optimization dimensions
	assert(x.size() == opt_dimension_);

	// define a optimization
	nlopt::opt optimizer(nlopt::LD_SLSQP, opt_dimension_);

	// set lower bound and upper bound
	build_bounds(x, opt_lb_, opt_ub_);

	optimizer.set_lower_bounds(opt_lb_);
	optimizer.set_upper_bounds(opt_ub_);

	// set boundary parameters
	optimizer.set_xtol_rel(1e-3);
	optimizer.set_ftol_abs(1e-3);
        optimizer.set_maxeval(1000);
	optimizer.set_maxtime(0.05);

	// set objective function
	optimizer.set_min_objective(objective, &policy_reference_);

	// set constraints for equality
	double command = reference_command;
	optimizer.add_equality_constraint(equality_constraint, &command, 1e-3);

	// optimization process
	double minimum = 0;
	try {
		optimizer.optimize(x, minimum);
	} catch (const std::exception& e) {
		// return coarse solution
		*solution = std::move(guess_solution);
		std::cout << "NLopt failed: " << e.what() << std::endl;
		return true;
	}

	// return planning results
	std::vector<std::pair<uint32_t, float>> result;
	for (size_t i = 0; i < opt_dimension_; ++i) {
		uint32_t index = items_config_.at(i).index;
		result.emplace_back(std::make_pair(index, x.at(i)));
	}
	*solution = std::move(result);
	return true;
}

bool NonlinearOptimization::init_reference_allocation_lookup(const std::vector<float>& current_state, const float reference_command)
{
	allocation_type_ = original_config_.allocation_type();

	// update current allocation states
	policy_reference_.clear();
	policy_reference_.reserve(opt_dimension_);

	switch (allocation_type_)
	{
	case AllocationType::PROPORTIONAL_ALLOCATION:
		{
			for (size_t i = 0; i < opt_dimension_; i++) {
				const float u = items_config_.at(i).assigned_factor * reference_command;
				policy_reference_.emplace_back(u);
			}
		}
		break;
	case AllocationType::MARGIN_ALLOCATION:
		{
			std::vector<std::pair<float, float>> xv_pairs;
			xv_pairs.reserve(opt_dimension_);
			for (size_t i = 0; i < opt_dimension_; i++) {
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


			const float allocation = reference_command - sum_x;
			for (size_t i = 0; i < opt_dimension_; i++) {
				float allocation_delta = 0;
				if (reference_command > sum_x) {
					allocation_delta = allocation * (xv_pairs[i].second - xv_pairs[i].first) / sum_v_x;
				} else {
					allocation_delta = allocation * xv_pairs[i].first / sum_x;
				}
				policy_reference_.emplace_back(allocation_delta + xv_pairs[i].first);
			}
		}
		break;
	default:
		break;
	}

	return true;
}

double NonlinearOptimization::objective(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
	std::vector<float> * reference = (std::vector<float>*)data;
	double sum  = 0.0;
	assert(x.size() == reference->size());

	if (!grad.empty()) {
		grad.clear();
		for (std::size_t i = 0; i < x.size(); i++) {
			grad.emplace_back( 2 * (x[i] - reference->at(i)));
		}
	}

	for (std::size_t i = 0; i < x.size(); i++) {
		sum += (reference->at(i) - x.at(i)) * (reference->at(i) - x.at(i));
	}
	return sum;
}

double NonlinearOptimization::inequality_constraint(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
	std::tuple<uint32_t, double, double> * interval = (std::tuple<uint32_t, double, double> *)data;
	uint32_t index = std::get<0>(*interval);
	double a = std::get<1>(*interval);
	double b = std::get<2>(*interval);
	double o = 0.5 * (a + b);
	double r = 0.5 * (b - a);
	return r - std::abs(x[index] - o);
}

double NonlinearOptimization::equality_constraint(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
	double *u = (double*)data;

	if (!grad.empty()) {
		grad.clear();
		for (std::size_t i = 0; i < x.size(); i++) {
			grad.emplace_back(1.0);
		}
	}

	double sum = 0.0;
	for (const auto& xi : x) {
		sum += xi;
	}
	return sum - (*u);
}


} // namespace grid_tied_allo
} // ai_decision
