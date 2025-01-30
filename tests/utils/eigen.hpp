#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__EIGEN_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__EIGEN_HPP_

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

/// Meta function checking that T is a Eigen::Matrix<> (i.e. dimension is 2)
template <typename T, typename = void>
struct IsEigenMatrix : std::false_type {};

template <typename T>
struct IsEigenMatrix<T, std::void_t<decltype(T::NumDimensions)>> {
  static constexpr bool value = (T::NumDimensions == 2);
};

template <typename T>
constexpr bool IsEigenMatrix_v = IsEigenMatrix<T>::value;

}  // namespace details

/**
 *  @brief Grow a Dynamic sized Eigen::Vector by \a inc elements (uninitialized)
 *
 *  @param[inout] vector Eigen vector we wish to grow
 *  @param[in] inc Number of elements we wish to add
 */
template <typename VectorType, typename...,
          std::enable_if_t<tests::utils::details::IsEigenVector_v<VectorType>,
                           bool> = true>
constexpr auto Grow(Eigen::PlainObjectBase<VectorType> &vector,
                    std::size_t inc) {
  vector.conservativeResize(vector.size() + inc);
}

/**
 *  @brief Grow a Dynamic sized Eigen::Matrix
 *
 *  @param[inout] vector Eigen matrix (NumDimensions == 2) we wish to grow
 *  @param[in] row_inc Number of rows we wish to add
 *  @param[in] col_inc Number of cols we wish to add (default to row_inc)
 */
template <typename MatrixType, typename...,
          std::enable_if_t<tests::utils::details::IsEigenMatrix_v<MatrixType>,
                           bool> = true>
constexpr auto Grow(Eigen::PlainObjectBase<MatrixType> &matrix,
                    std::size_t row_inc,
                    std::optional<std::size_t> col_inc = std::nullopt) {
  matrix.conservativeResize(matrix.rows() + row_inc,
                            matrix.cols() + col_inc.value_or(row_inc));
}

/**
 *  @brief Creates an OutVector from the concatenation of all given \a vectors
 *
 *  @note SFINAE: Only available for Vectors (NumDimension equals to 1)
 *
 *  @tparam OutVector The output vector type returned
 *  @tparam ...VectorTypes All input vectors
 *
 *  @param[in] ...vectors All vectors we wish to concatenates
 *
 *  @return OutVector With coefs set as the concatenation of all input vectors
 */
template <typename OutVector, typename... VectorTypes>
constexpr auto ConcatenateAs(const Eigen::DenseBase<VectorTypes> &...vectors)
    -> std::enable_if_t<details::IsEigenVector_v<OutVector> and
                            (details::IsEigenVector_v<VectorTypes> and ...),
                        OutVector> {
  // NOTE: Couldn't find a way to use the fold expression with the Eigen
  // stream + comma operators initialization shenanigans...
  // 'out << (vectors, ...);' doesn't work ??
  // Therefore we use the segment(i, n) with i begin updated through the comma
  // operator, which should be the same as 'out << A, B, C, ...;'

  OutVector out;
  out.resize((0 + ... + vectors.size()));

  std::size_t i = 0;
  // NOTE: the void(...) here is just because of comma operator of Eigen...
  ((void(out.segment(i, vectors.size()) = vectors), i += vectors.size()), ...);

  // This evaluates to:
  // out.segment(i, vectors[0].size()) = vectors[0];
  // i += vectors[0].size();
  // out.segment(i, vectors[1].size()) = vectors[1];
  // i += vectors[1].size();
  //  ...
  // out.segment(i, vectors[N].size()) = vectors[N];
  // i += vectors[N].size();

  return out;
}

/// Inside a vector, represents a strip using the expected number of
/// head/tail elements needed
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
  std::optional<Strip> strip = Strip{.head = 2, .tail = 2};

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

#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__EIGEN_HPP_
