
#include <chrono>
#include<thread>

#include <stdio.h>

#include "grid_tied_allocation.h"

using namespace ai_decision;
using namespace grid_tied_allocation;

int main(int argc, char** argv)
{
	// set configuration parameters
	OptimizerConfig config;
	config.clear_config();

	// set resolution
	config.allocation_resolution(15);
	// set type
	config.allocation_type(AllocationType::PROPORTIONAL_ALLOCATION);
	// set solve method
	config.enable_nlopt(true);

	// set parallel computing mode
	config.enable_multi_threads_in_dp(false);

	std::vector<OptimizerConfig::ItemConfig> items;
	OptimizerConfig::ItemConfig it;

	// 1
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 0;
	it.lower_bound = 0.0f;
	it.upper_bound =  250.0f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(0.0f, 30.0f));
	it.resonances.push_back(std::make_pair(50.0f, 110.0f));
	items.push_back(it);

	// 2
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 1;
	it.lower_bound = 0;
	it.upper_bound =  200.0f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(0.0f, 40.0f));
	it.resonances.push_back(std::make_pair(50.0f, 60.0f));
	items.push_back(it);

	// 3
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 2;
	it.lower_bound = 0;
	it.upper_bound =  150.0f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(10.0f, 30.f));
	it.resonances.push_back(std::make_pair(60.f, 100.f));
	items.push_back(it);

	// 4
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 3;
	it.lower_bound = 0;
	it.upper_bound =  180.0f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(20.f, 50.f));
	it.resonances.push_back(std::make_pair(70.f, 120.f));
	items.push_back(it);

	// 5
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 4;
	it.lower_bound = 0;
	it.upper_bound =  200.f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(0.f, 20.f));
	it.resonances.push_back(std::make_pair(40.f, 130.f));
	items.push_back(it);

	// 6
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 5;
	it.lower_bound = 0.0f;
	it.upper_bound =  150.f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(0.f, 10.f));
	it.resonances.push_back(std::make_pair(30.f, 50.f));
	items.push_back(it);

	// 7
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 6;
	it.lower_bound = 0.f;
	it.upper_bound =  200.f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(20.f, 40.f));
	it.resonances.push_back(std::make_pair(70.f, 100.f));
	items.push_back(it);

	// 8
	it.assigned_factor = 0.125f;
	it.enabled = true;
	it.index = 7;
	it.lower_bound = 0.f;
	it.upper_bound =  400.f;
	it.resonances.clear();
	it.resonances.push_back(std::make_pair(20.f, 60.f));
	it.resonances.push_back(std::make_pair(90.f, 110.f));
	items.push_back(it);

	// load resources
	config.items_config(items);

	// create algorithm
	const std::vector<float> current_state{100, 80, 45, 10, 100, 70, 10, 180};
	const std::vector<float> commands{400.f, 800.f, 1200.f, 1600.f, 1500.f, 1000.f, 600.f, 1300.f, 1730.0f};
	//const std::vector<float> commands{1730.0f};

	std::vector<std::pair<uint32_t, float>> allocation_result;

	// create allocation algorithm
	GridTiedAllocation opt(config);

	std::vector<float> x = std::move(current_state);
	for (auto& u : commands) {
		std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

		allocation_result.clear();
		opt.process(x, u, &allocation_result);

		std::chrono::steady_clock::time_point tf = std::chrono::steady_clock::now();

		std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(tf - t0);

		x.clear();
		printf("expect allocation: %f, ", u);
		float total_u = 0;
		for (auto& v : allocation_result) {
			x.push_back(v.second);
			total_u += v.second;
			printf("%f ", v.second);
		}
		printf("consumed_time: %f(ms) ; real allocation: %f\n", 1000 * static_cast<double>(time_span.count()), total_u);

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	return 0;

}
