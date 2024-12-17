#pragma once

#include <array>
#include <ostream>
#include <type_traits>
#include <utility>  // std::forward

namespace tests::utils {

namespace details {

/// Meta function use to determine if T is 'streamable', i.e. the operation
/// 'std::ostream & << T' exists
template <typename T, typename = void>
struct IsStreamable : std::false_type {};

template <typename T>
struct IsStreamable<T, std::void_t<decltype(std::declval<std::ostream &>()
                                            << std::declval<T>())>>
    : std::true_type {};

template <typename T>
constexpr bool IsStreamable_v = IsStreamable<T>::value;

}  // namespace details

/**
 *  \brief Generate a std::array of variable size with a fixed type
 *
 *  This function may be used when people want to create let the compiler
 *  deduces the size of an array<> based on the number of argument provided at
 *  construction but force the type contained within the array since it
 *  cast of each values provided into the ValueType.
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

/**
 *  \brief If possible, prints \a value to \a os, otherwise prints \a backup
 *
 *  \param[in] value Value we wish to print
 *  \param[inout] os Output stream we wish to output \a value in
 *  \param[in] backup Backup string used when value is not streamable
 *
 *  \return std::ostream& The ostream used
 */
template <typename T>
constexpr auto TryToPrintTo(T &&value, std::ostream &os,
                            std::string_view backup = "<Not Printable>")
    -> void {
  if constexpr (details::IsStreamable_v<std::remove_cv_t<decltype(value)>>) {
    os << std::forward<T>(value);
  } else {
    os << backup;
  }
}

}  // namespace tests::utils
