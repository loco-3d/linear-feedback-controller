#pragma once

#include <cstddef>  // std::size_t
#include <optional>
#include <ostream>
#include <type_traits>

#include "Eigen/Core"

namespace tests::utils {

namespace details {

/// Meta function checking that T is a Eigen::Vector<> (i.e. dimension is 1)
template <typename T, typename = void>
struct IsEigenVector : std::false_type {};

template <typename T>
struct IsEigenVector<T, std::void_t<decltype(T::NumDimensions)>> {
  static constexpr bool value = (T::NumDimensions == 1);
};

template <typename T>
constexpr bool IsEigenVector_v = IsEigenVector<T>::value;

}  // namespace details

/// Inside a vector, represents a strip using the expected number of head/tail
/// elements needed
struct Strip {
  std::size_t head; /*!< Number of elements at the head of the vector */
  std::size_t tail; /*!< Number of elements at the tail of the vector */
};

/// Print format data structure use to control formating of PrintTo
struct VectorPrintFormat {
  /// This is forwarded to Eigen::IOFormat
  Eigen::IOFormat io_fmt = {
      Eigen::StreamPrecision,  // precision
      0,                       // flags
      " ",                     // coeffSeparator
      " ",                     // rowSeparator
      "",                      // rowPrefix
      "",                      // rowSuffix
      "",                      // matPrefix
      "",                      // matSuffix
      ' ',                     // fill
  };

  /// Shorten the number of elements printed
  std::optional<Strip> strip = std::nullopt;

  /// Add the size of the vector at the beginning
  bool with_size = true;
};

}  // namespace tests::utils

// Inside Eigen namespace for ADL
namespace Eigen {

/**
 *  \brief Print \a vector into \a os, given \a fmt
 *
 *  Prints the vector using 'Vector{.data = <...>}' format, with <...> begin the
 *  result of the default eigen stream operator (given the io_fmt provided).
 *
 *  When adding '.with_size = true' to the format, it prepends '.data = <...>'
 *  with '.size = N, '.
 *
 *  Striping shorten the number of elements printed and only prints the first
 *  'strip.head' and last 'strip.tail' elements, separated by '...'.
 *
 *  The default format prints the vector with its size, in a row, separated by
 *  white space and stiped in order to only show the first and last 2 elements.
 *
 *  \param[in] vector The vector we wish to print
 *  \param[inout] os Ptr to the ostream we wish to print to
 *  \param[in] fmt The vector format used when printing
 */
template <typename VectorType, typename...,
          std::enable_if_t<tests::utils::details::IsEigenVector_v<VectorType>,
                           bool> = true>
auto PrintTo(const Eigen::DenseBase<VectorType> &vector, std::ostream *os,
             tests::utils::VectorPrintFormat fmt = {}) -> void {
  if (os == nullptr) return;

  *os << "Vector{";

  if (fmt.with_size) {
    *os << ".size() = " << vector.size() << ", ";
    *os << ".data = ";
  }

  if (fmt.strip and (vector.size() > (fmt.strip->head + fmt.strip->tail))) {
    *os << vector.head(fmt.strip->head).format(fmt.io_fmt);
    *os << fmt.io_fmt.rowSeparator << "..." << fmt.io_fmt.rowSeparator;
    *os << vector.tail(fmt.strip->tail).format(fmt.io_fmt);
  } else {
    *os << vector.format(fmt.io_fmt);
  }

  *os << '}';
}

}  // namespace Eigen
