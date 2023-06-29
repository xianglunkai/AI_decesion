#pragma once

#include <stdlib.h>
#include <type_traits>
#include <future>
#include <functional>

namespace ai_decision {
namespace grid_tied_allocation {

// used to forbid copy of objects
#define CLASS_NO_COPY(c) c(const c &other) = delete; c &operator=(const c&) = delete

template <typename F, typename... Args>
static auto Async(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
  return std::async(
                   std::launch::async,
                   std::bind(std::forward<F>(f), std::forward<Args>(args)...));
}

} // namespace grid_tied_allo
} // ai_decision
