#pragma once

#include <array>
#include <ostream>
#include <type_traits>
#include <utility>  // std::forward

namespace test::utils {

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

namespace meta {

template <typename T, typename = void>
struct IsStreamable : std::false_type {};

template <typename T>
struct IsStreamable<T, std::void_t<decltype(std::declval<std::ostream &>()
                                            << std::declval<T>())>>
    : std::true_type {};

template <typename T>
constexpr bool IsStreamable_v = IsStreamable<T>::value;

}  // namespace meta

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
  if constexpr (meta::IsStreamable_v<std::remove_cv_t<decltype(value)>>) {
    os << std::forward<T>(value);
  } else {
    os << backup;
  }
}

/**
 *  \brief Represents a temporary mutation of a given reference
 *
 *  RAII (https://en.cppreference.com/w/cpp/language/raii) object use to
 *  automatically modify a reference when constructing the mutator and assigning
 *  back the old value to it when the mutator destroyed.
 *
 *  Additionnaly, the mutator can be aborted, reseted or we can even triggers
 *  the revert by hand when needed through the appropriate methods.
 *
 *  Example: https://godbolt.org/z/eTfsvvobc
 *
 *  \tparam ValueType The underlying type of the value we wish to mutate
 */
template <typename _ValueType>
struct Mutation {
  /// Underlying ValueType stored inside the mutation
  using ValueType = _ValueType;

  /// Default Ctor (not allowed)
  constexpr Mutation() = delete;

  /**
   *  \brief Construct a Mutation from a reference and a new tmp value
   *
   *  \tparam T Type of the temporary data assigned to \a value
   *
   *  \param[in] value Reference to the data we wish to mutate
   *  \param[in] tmp Value that will mutate the data
   */
  template <typename T>
  constexpr Mutation(ValueType &value, T &&tmp)
      : Mutation(std::addressof(value), std::forward<T>(tmp)) {}

  /// Move Ctor
  constexpr Mutation(Mutation &&other) {
    // Use the move assignement below
    *this = std::move(other);
  }

  /// Move Assignement
  constexpr auto operator=(Mutation &&other) -> Mutation & {
    ptr_ = other.ptr_;
    old_value_ = std::move(other.old_value_);

    other.Abort();
    return *this;
  }

  /// Copy Ctor (not allowed)
  constexpr Mutation(const Mutation &other) = delete;

  /// Copy Assignement (not allowed)
  constexpr Mutation &operator=(const Mutation &other) = delete;

  /// Dtor that triggers the revert operation
  virtual ~Mutation() { Revert(); }

  /**
   *  \brief Reset the mutator with a new data/tmp value to look at
   *
   *  \param[in] new_data New data to look for
   *  \param[in] tmp New temporary value to assign to \a new_data
   *  \param[in] revert True if we wish to revert the previous data before
   */
  template <typename T>
  constexpr auto Reset(ValueType &new_data, T &&tmp, bool revert = true)
      -> void {
    if (revert) {
      Revert();
    }

    *this = Mutation{new_data, std::forward<T>(tmp)};
  }

  /**
   *  \brief Abort the mutation
   */
  constexpr auto Abort() noexcept -> void { ptr_ = nullptr; }

  /**
   *  \return True when the mutation is aborted
   */
  constexpr auto IsAborted() const noexcept -> bool { return ptr_ == nullptr; }

  /**
   *  \brief Triggers the assignement of the old value into data, if not aborted
   */
  constexpr auto Revert() -> void {
    if (not IsAborted()) {
      *ptr_ = old_value_;
    }
  }

  /**
   *  \return ValueType * const Current data ptr being watched by the mutator
   */
  constexpr auto GetData() const noexcept -> ValueType *const { return ptr_; }

  /**
   *  \return const ValueType& Old value memorized (use when reverting)
   */
  constexpr auto GetOldValue() const noexcept -> const ValueType & {
    return old_value_;
  }

  /**
   *  \return ValueType& Old value memorized (use when reverting)
   */
  constexpr auto GetOldValue() noexcept -> ValueType & { return old_value_; }

 private:
  /// Private ctor (/!\ Assumes that \a ptr is NOT NULL /!\)
  template <typename T>
  constexpr Mutation(ValueType *ptr, T &&tmp) : ptr_(ptr), old_value_(*ptr_) {
    *ptr_ = std::forward<T>(tmp);
  }

  ValueType *ptr_;      /*!< Ptr of the data we wish to mutate/revert */
  ValueType old_value_; /*!< Old value memorize before mutation */
};

/**
 *  \brief Helper function that creates a Mutation<>
 *
 *  This function only serves to clarify names by adding an emphasis on the
 *  'temporary' aspect of the Mutation.
 *
 *  \tparam ValueType Underlying type of the data we wish the mutate
 *  \tparam T Type of the temporary assigned to \a value
 *
 *  \param[in] value The value we wish to temporarly mutate
 *  \param[in] tmp The new temporary value
 *  \return Mutation<ValueType> Containing the mutation, reverting the changes
 *                              when going out of scope
 */
template <typename ValueType, typename T>
constexpr auto TemporaryMutate(ValueType &value, T &&tmp)
    -> Mutation<ValueType> {
  return Mutation<ValueType>{value, std::forward<T>(tmp)};
}

/// Format specifier for the PrintTo function below
struct MutationPrintFormat {
  unsigned show_data : 1;      /*!< Print the Mutation.GetData() value */
  unsigned show_old_value : 1; /*!< Print the Old value */
};

/**
 *  \brief Prints \a mutation to the provided \a os, given the format \a fmt
 */
template <typename ValueType>
constexpr auto PrintTo(const Mutation<ValueType> &mutation,
                       std::ostream *const os,
                       MutationPrintFormat fmt = {
                           .show_data = true,
                           .show_old_value = true,
                       }) -> void {
  if (os != nullptr) {
    *os << "Mutation{";

    if (mutation.IsAborted()) {
      *os << " ABORTED ";
    } else {
      *os << ".ptr = " << (void *const)mutation.GetData();

      if (fmt.show_data) {
        *os << " -> ";
        TryToPrintTo(*mutation.GetData(), *os);
      }

      if (fmt.show_old_value) {
        *os << ", .old_value = ";
        TryToPrintTo(mutation.GetOldValue(), *os);
      }
    }

    *os << "}";
  }
}
}  // namespace test::utils
