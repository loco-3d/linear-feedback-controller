#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__PRINT_TO_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__PRINT_TO_HPP_

#include <ostream>
#include <string_view>
#include <type_traits>
#include <utility>  // std::forward

namespace tests::utils {

namespace details {

/// [SFINAE] Check if `std::ostream & << T{}` is valid
template <typename T, typename = void>
struct IsStreamable : std::false_type {};

template <typename T>
struct IsStreamable<T, std::void_t<decltype(std::declval<std::ostream&>()
                                            << std::declval<T>())>>
    : std::true_type {};

template <typename T>
constexpr bool IsStreamable_v = IsStreamable<T>::value;

/// [SFINAE] Check if `PrintTo(T{}, std::ostream*, Others{}...)` function exists
template <typename T, typename = void, typename... Others>
struct HasPrintTo : std::false_type {};

template <typename T, typename... Others>
struct HasPrintTo<T,
                  std::void_t<decltype(PrintTo(std::declval<T>(),
                                               std::declval<std::ostream*>(),
                                               std::declval<Others>()...))>,
                  Others...> : std::true_type {};

template <typename T, typename... Others>
constexpr bool HasPrintTo_v = HasPrintTo<T, Others...>::value;

/// [SFINAE] Check if `T::PrintFormat` exists
template <typename T, typename = void>
struct HasPrintFormat : std::false_type {};

template <typename T>
struct HasPrintFormat<T, std::void_t<typename T::PrintFormat>>
    : std::true_type {};

template <typename T>
constexpr bool HasPrintFormat_v = HasPrintFormat<T>::value;

/// [SFINAE] Return T::PrintFormat if HasFormat = true, else use U
template <bool HasFormat, typename T, typename U>
struct PrintFormatOfImpl;

template <typename T, typename U>
struct PrintFormatOfImpl<true, T, U> {
  using type = typename T::PrintFormat;
};

template <typename T, typename U>
struct PrintFormatOfImpl<false, T, U> {
  using type = U;
};

template <typename T, typename U>
struct PrintFormatOf : PrintFormatOfImpl<HasPrintFormat_v<T>, T, U> {};

template <typename T, typename U>
using PrintFormatOf_t = typename PrintFormatOf<T, U>::type;

}  // namespace details

/// Dummy empty data struct used by PrintFormatOf<T>
struct PrintFormatDoesNotExists {};

/**
 *  @brief Either T::PrintFormat (if it exists) or an empty dummy type
 */
template <typename T>
using PrintFormatOf = details::PrintFormatOf_t<T, PrintFormatDoesNotExists>;

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
constexpr auto TryToPrintTo(T&& value, std::ostream* os,
                            std::string_view backup = "<Not Printable>")
    -> void {
  if (os == nullptr) return;
  if constexpr (details::IsStreamable_v<std::remove_cv_t<decltype(value)>>) {
    *os << std::forward<T>(value);
  } else {
    *os << backup;
  }
}

}  // namespace tests::utils

#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__PRINT_TO_HPP_
