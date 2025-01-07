#pragma once

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
