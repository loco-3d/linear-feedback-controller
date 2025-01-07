#pragma once

#include <array>
#include <utility>  // std::forward

namespace tests::utils {

/**
 *  \brief Generate a std::array of variable size with a fixed type
 *
 *  Equivalent to c++20 to std::to_array()
 *  (https://en.cppreference.com/w/cpp/container/array/to_array), but worst ;)
 *
 *  This function may be used when people want to let the compiler deduces the
 *  size of an array<> based on the number of argument provided at construction
 *  but force the type contained within the array since it implicitely casts
 *  each values provided into the ValueType.
 *
 *  Example: https://godbolt.org/z/qE84bKrdx
 *
 *  \tparam ValueType The array value_type needed
 *  \tparam ...T Deduces types of the values forwarded
 *
 *  \param ...values Initial values used to construct the array
 *  \return std::array<ValueType, sizeof...(T)> The array initialized
 */
template <typename ValueType, typename... T>
constexpr auto MakeArray(T &&...values) -> std::array<ValueType, sizeof...(T)> {
  return std::array<ValueType, sizeof...(T)>{
      ValueType{std::forward<T>(values)}...};
}

}  // namespace tests::utils
