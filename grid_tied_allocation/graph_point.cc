/**
 * @file: graph_point.cc
 **/

#include "graph_point.h"

namespace ai_decision {
namespace grid_tied_allocation {

std::uint32_t STGraphPoint::index_s() const { return index_s_; }
std::uint32_t STGraphPoint::index_t() const { return index_t_; }

const STPoint& STGraphPoint::point() const { return point_; }
const STGraphPoint* STGraphPoint::pre_point() const { return pre_point_; }

float STGraphPoint::reference_cost() const { return reference_cost_; }
float STGraphPoint::obstacle_cost() const { return obstacle_cost_; }
float STGraphPoint::total_cost() const { return total_cost_; }

void STGraphPoint::init(const std::uint32_t index_t,
                        const std::uint32_t index_s, const STPoint& st_point) {
	index_t_ = index_t;
	index_s_ = index_s;
	point_ = st_point;
}

void STGraphPoint::set_total_cost(const float total_cost) {
	total_cost_ = total_cost;
}

void STGraphPoint::set_pre_point(const STGraphPoint& pre_point) {
	pre_point_ = &pre_point;
}

} // namespace grid_tied_allo
} // ai_decision
