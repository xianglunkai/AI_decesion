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


} // namespace grid_tied_allo
} // ai_decision
