#pragma once

#include <ostream>
#include <string_view>
#include <type_traits>

#include "print_to.hpp"

namespace tests::utils {

namespace details {

/// [SFINAE] Check if T define begin(T{}) and end(T{}) functions
template <typename T, typename = void>
struct IsIterable : std::false_type {};

template <typename T>
struct IsIterable<T, std::void_t<decltype(begin(std::declval<T>())),
                                 decltype(end(std::declval<T>()))>>
    : std::true_type {};

template <typename T>
constexpr bool IsIterable_v = IsIterable<T>::value;

/// [SFINAE] Check if T define size(T{}) function
template <typename T, typename = void>
struct HasSize : std::false_type {};

template <typename T>
struct HasSize<T, std::void_t<decltype(size(std::declval<T>()))>>
    : std::true_type {};

template <typename T>
constexpr bool HasSize_v = HasSize<T>::value;

/// [SFINAE] Check if T define ssize(T{}) function
template <typename T, typename = void>
struct HasSignedSize : std::false_type {};

template <typename T>
struct HasSignedSize<T, std::void_t<decltype(ssize(std::declval<T>()))>>
    : std::true_type {};

template <typename T>
constexpr bool HasSignedSize_v = HasSignedSize<T>::value;

}  // namespace details

template <typename Container,
          std::enable_if_t<details::IsIterable_v<Container>, bool> = true>
struct View {
  using IteratorType = decltype(begin(std::declval<Container>()));
  using Reference = decltype(*std::declval<IteratorType>());
  using ValueType = std::remove_reference_t<Reference>;

  constexpr explicit View(const Container& c) noexcept : m_c(&c) {}
  constexpr auto operator=(const Container& c) noexcept -> View& {
    m_c = &c;
    return *this;
  }

  friend constexpr auto begin(View v) { return begin(*v.m_c); }

  friend constexpr auto end(View v) { return end(*v.m_c); }

  friend constexpr auto ssize(View v) {
    if constexpr (details::HasSignedSize_v<Container>) {
      return ssize(*v.m_c);
    } else {
      return distance(begin(v), end(v));
    }
  }

  friend constexpr auto size(View v)
      -> std::enable_if_t<details::HasSize_v<Container>, std::size_t> {
    return size(*v.m_c);
  }

  struct PrintFormat {
    std::string_view header = "[";
    std::string_view separator = ", ";
    std::string_view footer = "]";

    PrintFormatOf<ValueType> value_fmt = {};

    bool with_size = false;
    bool with_index = false;
  };

  friend constexpr auto PrintTo(View view, std::ostream* os,
                                PrintFormat fmt = {}) -> void {
    if (os == nullptr) return;

    if (fmt.with_size) {
      *os << "{.size = " << size(view) << ", data -> ";
    }

    *os << fmt.header;

    std::size_t i = 0;
    for (const auto& val : view) {
      if (fmt.with_index) {
        *os << '[' << i++ << "]: ";
      }

      if constexpr (details::HasPrintTo_v<ValueType, decltype(fmt.value_fmt)>) {
        PrintTo(val, os, fmt.value_fmt);
      } else if constexpr (details::HasPrintTo_v<ValueType>) {
        PrintTo(val, os);
      } else {
        TryToPrintTo(val, os);
      }
      *os << fmt.separator;
    }

    *os << fmt.footer;

    if (fmt.with_size) {
      *os << '}';
    }
  }

 private:
  Container const* const m_c;
};

}  // namespace tests::utils
