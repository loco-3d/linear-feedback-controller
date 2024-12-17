#pragma once

#include <cstddef>  // std::size_t
#include <optional>
#include <ostream>
#include <type_traits>

#include "Eigen/Core"

namespace tests::utils {

namespace details {

template <typename T, typename = void>
struct IsEigenVector : std::false_type {};

template <typename T>
struct IsEigenVector<T, std::void_t<decltype(T::NumDimensions)>> {
  static constexpr bool value = (T::NumDimensions == 1);
};

template <typename T>
constexpr bool IsEigenVector_v = IsEigenVector<T>::value;

}  // namespace details

struct Strip {
  std::size_t head;
  std::size_t tail;
};

struct VectorPrintFormat {
  Eigen::IOFormat io_fmt;  // This forwarded to Eigen::IOFormat

  std::optional<Strip> strip;
  bool with_size;
};

}  // namespace tests::utils

// Inside Eigen namespace for ADL
namespace Eigen {
template <typename VectorType, typename...,
          std::enable_if_t<tests::utils::details::IsEigenVector_v<VectorType>,
                           bool> = true>
auto PrintTo(const Eigen::DenseBase<VectorType> &vector, std::ostream *os,
             tests::utils::VectorPrintFormat fmt = {
                 .io_fmt =
                     {
                         Eigen::StreamPrecision,  // precision
                         0,                       // flags
                         " ",                     // coeffSeparator
                         " ",                     // rowSeparator
                         "",                      // rowPrefix
                         "",                      // rowSuffix
                         "",                      // matPrefix
                         "",                      // matSuffix
                         ' ',                     // fill
                     },
                 .strip = tests::utils::Strip{.head = 2, .tail = 2},
                 .with_size = true,
             }) -> void {
  if (os == nullptr) return;

  *os << "Vector{";

  if (fmt.with_size) {
    *os << ".size() = " << vector.size() << ", ";
  }

  *os << ".data = ";
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
