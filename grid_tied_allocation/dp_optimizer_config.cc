#include "dp_optimizer_config.h"

namespace ai_decision {
namespace grid_tied_allocation {

// singleton instance
DpOptimizerConfig *DpOptimizerConfig::singleton_;
DpOptimizerConfig *dp_optimizer_config()
{
	return DpOptimizerConfig::get_singleton();
}

// constructor functions
DpOptimizerConfig::DpOptimizerConfig()
{
	singleton_ = this;
}


} // namespace grid_tied_allo
} // ai_decision
