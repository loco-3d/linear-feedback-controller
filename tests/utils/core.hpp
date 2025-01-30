#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__CORE_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__CORE_HPP_

#include <tuple>
#include <utility>

namespace tests::utils {

/**
 *  @return A predicate functor calling the underlying predicate and
 *          returning it's negation
 *
 *  @param[in] pred A simple predicate taking any arguments and returning a bool
 */
template <typename Pred>
constexpr auto DoNot(Pred&& pred) {
  return [&](auto&&... arg) -> bool {
    return not pred(std::forward<decltype(arg)>(arg)...);
  };
}

/**
 *  @return A predicate functor returning true if ANY one of the underlying
 *          predicates returns true
 *
 *  @param[in] ...preds All predicates
 *
 *  @warning Predicates will take the exact same arguments
 */
template <typename... Preds>
constexpr auto AnyOf(Preds&&... preds) {
  return [&](const auto&... arg) -> bool { return (... or preds(arg...)); };
}

/**
 *  @return A predicate functor returning true if ALL the underlying predicates
 *          returns true
 *
 *  @param[in] ...preds All predicates
 *
 *  @warning Predicates will take the exact same arguments
 */
template <typename... Preds>
constexpr auto AllOf(Preds&&... preds) {
  return [&](const auto&... arg) -> bool { return (... and preds(arg...)); };
}

/**
 *  @return A predicate functor returning the result of calling \a pred by
 *          filtering the input args based on the indices provided
 *
 *  @param[in] ...preds All predicates
 */
template <std::size_t... Idx, typename Pred>
constexpr auto WithArgs(Pred&& pred) {
  return [&](auto&&... arg) {
    auto t = std::forward_as_tuple(std::forward<decltype(arg)>(arg)...);
    return pred(std::forward<decltype(std::get<Idx>(t))>(std::get<Idx>(t))...);
  };
}

}  // namespace tests::utils
#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__CORE_HPP_
