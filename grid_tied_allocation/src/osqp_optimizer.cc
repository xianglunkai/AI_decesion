#include "osqp_optimizer.h"

#include <osqp.h>


#include <algorithm>
#include <limits>
#include <string>
#include <iostream>
#include <cmath>
#include <assert.h>
#include <tuple>

namespace ai_decision {
namespace grid_tied_allocation {

OsqpOptimizer::OsqpOptimizer(const OptimizerConfig& config)
	: original_config_(config)
{
}


bool OsqpOptimizer::process(const std::vector<float>& current_state,
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

bool OsqpOptimizer::init_items_config(const float reference_command)
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

void OsqpOptimizer::build_bounds(const std::vector<double> &x, std::vector<double> &lb, std::vector<double> &ub)
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

bool OsqpOptimizer::retrieve_allocation_profile(const float reference_command, const std::vector<std::pair<uint32_t, float>>& guess_solution, std::vector<std::pair<uint32_t, float>>* const solution)
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

	float rou = 200.0f;

	// set P matrix in csc format
	const size_t p_data_size = opt_dimension_ * (opt_dimension_ + 1) / 2;
	OSQPFloat P_data[p_data_size];
	OSQPInt P_nnz = p_data_size;
	OSQPInt P_indices[p_data_size];
	OSQPInt P_indptr[opt_dimension_ + 1];

	size_t k = 0;
	for (size_t i = 0; i < opt_dimension_; ++i) {
		for (size_t j = 0; j < i; ++j) {
			P_data[k] = 2 * rou;
			P_indices[k] = j;
			k = k + 1;
		}
		P_data[k] = 2 * (1 + rou);
		P_indices[k] = i;
		k = k + 1;
	}
	for (size_t l = 0; l < opt_dimension_ ; ++l) {
		P_indptr[l] = l * (l + 1) / 2;
	}
	P_indptr[opt_dimension_] = p_data_size;


	// set q matrix in csc format
	OSQPFloat q[opt_dimension_];
	for (size_t i = 0; i < opt_dimension_; i++) {
		q[i] = -2.0f * (policy_reference_.at(i) + reference_command * rou);
	}


	// set A matrix in csc format
	const size_t A_data_size = opt_dimension_;
	OSQPFloat A_data[A_data_size];
	OSQPInt A_nnz = A_data_size;
	OSQPInt A_indices[A_data_size];
	OSQPInt A_indptr[opt_dimension_ + 1];

	for (size_t i = 0; i < opt_dimension_; i++) {

		A_indices[i] = i;
		A_data[i] = 1;
		A_indptr[i] = i;
	}
	A_indptr[opt_dimension_] = A_data_size;

	// set l and u
	// set lower bound and upper bound
	build_bounds(x, opt_lb_, opt_ub_);

	OSQPFloat l[opt_dimension_ + 1];
    	OSQPFloat u[opt_dimension_ + 1];
	for (int i = 0; i < opt_dimension_ ; ++i) {
		l[i] = opt_lb_[i];
		u[i] = opt_ub_[i];
	}

	// set m and n
	OSQPInt  n = opt_dimension_;
	OSQPInt  m = opt_dimension_;

	/* Exitflag */
	OSQPInt exitflag = 0;

	/* Solver, settings, matrices */
	OSQPSolver   *solver;
	OSQPSettings *settings;
	OSQPCscMatrix* P = reinterpret_cast<OSQPCscMatrix*>(malloc(sizeof(OSQPCscMatrix)));
	OSQPCscMatrix* A = reinterpret_cast<OSQPCscMatrix*>(malloc(sizeof(OSQPCscMatrix)));

	/* Populate matrices */
	csc_set_data(A, m, n, A_nnz, A_data, A_indices, A_indptr);
	csc_set_data(P, n, n, P_nnz, P_data, P_indices, P_indptr);

	/* Set default settings */
	settings = reinterpret_cast<OSQPSettings*>(malloc(sizeof(OSQPSettings)));
	if (settings) {
		osqp_set_default_settings(settings);
		settings->alpha = 1.0; /* Change alpha parameter */
		settings->eps_abs = 1.0e-5;
		settings->eps_rel = 1.0e-5;
		settings->max_iter = 1000;
		settings->polishing = true;
		settings->verbose = false;
	}

	/* Setup solver */
	exitflag = osqp_setup(&solver, P, q, A, l, u, m, n, settings);

	/* Solve problem */
    	if (!exitflag) exitflag = osqp_solve(solver);


	// return planning results
	std::vector<std::pair<uint32_t, float>> result;
	for (size_t i = 0; i < opt_dimension_; ++i) {
		const uint32_t index = items_config_.at(i).index;
		const float value = solver->solution->x[i];
		result.emplace_back(std::make_pair(index, value));
	}
	*solution = std::move(result);

	/* Cleanup */
	osqp_cleanup(solver);
	if (A) free(A);
	if (P) free(P);
	if (settings) free(settings);

	return true;
}

bool OsqpOptimizer::init_reference_allocation_lookup(const std::vector<float>& current_state, const float reference_command)
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




} // namespace grid_tied_allo
} // ai_decision
