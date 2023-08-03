/*
 *  @file: graph_point.h
 */
#pragma once

#include <limits>
#include <cstdint>

namespace ai_decision {
namespace grid_tied_allocation {

class STPoint {
	// x-axis: t  y-axis: s
public:
	STPoint() = default;
	STPoint(const float s, const uint32_t t) : s_(s), t_(t) {}

	float s() const { return s_; }
	std::uint32_t t() const { return t_; }

	void set_s(const float s) { s_ = s; }
	void set_t(const uint32_t t) { t_ = t; }
private:
	std::uint32_t t_;
	float s_;
};

class STGraphPoint {
public:
	std::uint32_t index_s() const;
	std::uint32_t index_t() const;

	const STPoint& point() const;
	const STGraphPoint* pre_point() const;

	float reference_cost() const;
	float obstacle_cost() const;
	float total_cost() const;

	void init(const std::uint32_t index_t, const std::uint32_t index_s,
		  const STPoint& st_point);

	// total cost
	void set_total_cost(const float total_cost);

	// set previous point
	void set_pre_point(const STGraphPoint& prev_point);
private:
	STPoint point_;
	const STGraphPoint* pre_point_ = nullptr;
	std::uint32_t index_s_{0};
	std::uint32_t index_t_{0};

	float reference_cost_{0.0f};
	float obstacle_cost_{0.0f};
	float total_cost_{std::numeric_limits<float>::infinity()};
};

} // namespace grid_tied_allo
} // ai_decision
