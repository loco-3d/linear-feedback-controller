#pragma once

#include <ostream>

#include "print_to.hpp"  // TryToPrintTo

namespace tests::utils {

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
struct [[nodiscard]] Mutation final {
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
      : Mutation(std::addressof(value), std::forward<T>(tmp)) {
    static_assert(not std::is_const_v<ValueType>);
  }

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
  /// IMPORTANT: not virtual -> Do not inherit
  ~Mutation() { Revert(); }

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
  [[nodiscard]] constexpr auto IsAborted() const noexcept -> bool {
    return ptr_ == nullptr;
  }

  /**
   *  \brief Triggers the assignement of the old value into data, if not aborted
   */
  constexpr auto Revert() -> void {
    if (not IsAborted()) {
      *ptr_ = old_value_;
    }
  }

  /**
   *  \return const ValueType& Old value memorized (use when reverting)
   */
  constexpr auto OldValue() const noexcept -> const ValueType & {
    return old_value_;
  }

  /**
   *  \return ValueType& Old value memorized (use when reverting)
   */
  constexpr auto OldValue() noexcept -> ValueType & { return old_value_; }

  /// Format specifier for the PrintTo function below
  struct PrintFormat {
    bool show_data = false;      /*!< Print the current mutated value */
    bool show_old_value = false; /*!< Print the Old value */
  };

  /**
   *  \brief Prints \a mutation to the provided \a os, given the format \a fmt
   *  \note We use friend function here to access the private .ptr_ data member
   */
  friend constexpr auto PrintTo(const Mutation &mutation,
                                std::ostream *const os, PrintFormat fmt = {})
      -> void {
    if (os == nullptr) return;

    *os << "Mutation{";
    if (mutation.IsAborted()) {
      *os << " ABORTED ";
    } else {
      *os << ".ptr = " << (void *)mutation.ptr_;

      if (fmt.show_data) {
        *os << " -> ";
        TryToPrintTo(*mutation.ptr_, *os);
      }

      if (fmt.show_old_value) {
        *os << ", .old_value = ";
        TryToPrintTo(mutation.OldValue(), *os);
      }
    }
    *os << "}";
  }

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

}  // namespace tests::utils
