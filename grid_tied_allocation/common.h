#pragma once

#include <stdlib.h>
#include <type_traits>

namespace ai_decision {
namespace grid_tied_allocation {

// used to forbid copy of objects
#define CLASS_NO_COPY(c) c(const c &other) = delete; c &operator=(const c&) = delete

} // namespace grid_tied_allo
} // ai_decision
