#include "common.h"

namespace ai_decision {
namespace grid_tied_allocation {
/*
 * is_equal(): Integer implementation, provided for convenience and
 * compatibility with old code. Expands to the same as comparing the values
 * directly
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2)
{
    typedef typename std::common_type<Arithmetic1, Arithmetic2>::type common_type;
    return static_cast<common_type>(v_1) == static_cast<common_type>(v_2);
}

/*
 * is_equal(): double/float implementation - takes into account
 * std::numeric_limits<T>::epsilon() to return if 2 values are equal.
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2)
{
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
    typedef typename std::common_type<Arithmetic1, Arithmetic2>::type common_type;
    typedef typename std::remove_cv<common_type>::type common_type_nonconst;
    if (std::is_same<double, common_type_nonconst>::value) {
        return fabs(v_1 - v_2) < std::numeric_limits<double>::epsilon();
    }
#endif
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wabsolute-value"
    // clang doesn't realise we catch the double case above and warns
    // about loss of precision here.
    return fabsf(v_1 - v_2) < std::numeric_limits<float>::epsilon();
#pragma clang diagnostic pop
}

template bool is_equal<int>(const int v_1, const int v_2);
template bool is_equal<short>(const short v_1, const short v_2);
template bool is_equal<long>(const long v_1, const long v_2);
template bool is_equal<float>(const float v_1, const float v_2);
template bool is_equal<double>(const double v_1, const double v_2);


} // namespace grid_tied_allo
} // ai_decision

