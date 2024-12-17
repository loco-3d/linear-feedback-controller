#pragma once

#include <optional>
#include <ostream>
#include <string_view>
#include <tuple>

#include "eigen.hpp"  // PrintTo(DenseBase<>)
#include "linear_feedback_controller/pd_controller.hpp"

namespace tests::utils {

// Gains ////////////////////////////////////////////////////////////////////
struct Gains {
  using IdxType = Eigen::Index;

  Eigen::VectorXd p;
  Eigen::VectorXd d;

  inline static auto Random(IdxType p_size,
                            std::optional<IdxType> d_size = std::nullopt)
      -> Gains {
    return Gains{
        .p = Eigen::VectorXd::Random(p_size),
        .d = Eigen::VectorXd::Random(d_size.value_or(p_size)),
    };
  }
};

/// Function used because I keep miss using the API and inverting d/p
inline auto SetTo(linear_feedback_controller::PDController &pd_ctrl,
                  const Gains &gains) {
  return pd_ctrl.set_gains(gains.p, gains.d);
}

constexpr auto PrintTo(const Gains &gains, std::ostream *os) -> void {
  if (os == nullptr) return;

  *os << "Gains{";

  for (auto &&[name, gain] : {
           std::forward_as_tuple('p', gains.p),
           std::forward_as_tuple('d', gains.d),
       }) {
    *os << '.' << name << " = ";
    PrintTo(gain, os);
    *os << ", ";
  }

  *os << "}";
}

constexpr auto operator<<(std::ostream &os, const Gains &gains)
    -> std::ostream & {
  PrintTo(gains, &os);
  return os;
}

// References ///////////////////////////////////////////////////////////////
struct References {
  using IdxType = Eigen::Index;

  Eigen::VectorXd tau;
  Eigen::VectorXd q;

  inline static auto Random(IdxType tau_size,
                            std::optional<IdxType> q_size = std::nullopt)
      -> References {
    return References{
        .tau = Eigen::VectorXd::Random(tau_size),
        .q = Eigen::VectorXd::Random(q_size.value_or(tau_size)),
    };
  }
};

/// Function used because I keep miss using the API and inverting tau/q
inline auto SetTo(linear_feedback_controller::PDController &pd_ctrl,
                  const References &ref) {
  return pd_ctrl.set_reference(ref.tau, ref.q);
}

constexpr auto PrintTo(const References &ref, std::ostream *os) -> void {
  if (os == nullptr) return;

  *os << "References{";

  using namespace std::string_view_literals;
  for (auto &&[name, ref] : {
           std::forward_as_tuple("tau"sv, ref.tau),
           std::forward_as_tuple("q"sv, ref.q),
       }) {
    *os << '.' << name << " = ";
    PrintTo(ref, os);
    *os << ", ";
  }

  *os << "}";
}

constexpr auto operator<<(std::ostream &os, const References &references)
    -> std::ostream & {
  PrintTo(references, &os);
  return os;
}

}  // namespace tests::utils
