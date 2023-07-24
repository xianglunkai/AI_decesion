#pragma once

#include <stdlib.h>
#include <type_traits>
#include <future>
#include <functional>
#include <float.h>
#include <cmath>
#include <limits>
#include <stdint.h>
#include <type_traits>

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

/*
 * Check whether two floats are equal
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

/*
 * @brief: Check whether a float is zero
 */
inline bool is_zero(const float x) {
    return fabsf(x) < FLT_EPSILON;
}

/*
 * @brief: Check whether a double is zero
 */
inline bool is_zero(const double x) {
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
  return fabs(x) < FLT_EPSILON;
#else
  return fabsf(static_cast<float>(x)) < FLT_EPSILON;
#endif
}

/*
 * @brief: Check whether a float is zero
 */
template <typename T>
inline bool is_zero(const T fVal1) {
    static_assert(std::is_floating_point<T>::value,
                  "Template parameter not of type float");
    return is_zero(static_cast<float>(fVal1));
}

/*
 * @brief: Check whether a float is greater than zero
 */
template <typename T>
inline bool is_positive(const T fVal1) {
    static_assert(std::is_floating_point<T>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) >= FLT_EPSILON);
}


/*
 * @brief: Check whether a float is less than zero
 */
template <typename T>
inline bool is_negative(const T fVal1) {
    static_assert(std::is_floating_point<T>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
}

/*
 * @brief: Check whether a double is greater than zero
 */
inline bool is_positive(const double fVal1) {
    return (fVal1 >= static_cast<double>(FLT_EPSILON));
}

/*
 * @brief: Check whether a double is less than zero
 */
inline bool is_negative(const double fVal1) {
    return (fVal1 <= static_cast<double>((-1.0 * FLT_EPSILON)));
}


} // namespace grid_tied_allo
} // ai_decision
