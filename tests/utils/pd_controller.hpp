#ifndef LINEAR_FEEDBACK_CONTROLLER_TESTS__PD_CONTROLLER_HPP_
#define LINEAR_FEEDBACK_CONTROLLER_TESTS__PD_CONTROLLER_HPP_

#include <optional>
#include <ostream>

#include "eigen.hpp"  // PrintTo(DenseBase<>)
#include "linear_feedback_controller/pd_controller.hpp"

namespace tests::utils {

// Gains ////////////////////////////////////////////////////////////////////

/// Data structure to store pd controller gains for the unit tests
struct Gains {
  using IdxType = Eigen::Index;

  Eigen::VectorXd p;
  Eigen::VectorXd d;

  /**
   *  \brief Creates a Gains with random values in it
   *
   *  \param[in] p_size Size of the p vector needed
   *  \param[in] d_size Optional size of the d vector (default to p_size)
   *
   *  \return return type
   */
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

/**
 *  \brief Prints \a gains to \a os
 *
 *  Use the following format:
 *  Gains{.p = Vector{...}, .p = Vector{...}}
 *
 *  \param[in] gains The gains we wish to print
 *  \param[inout] os The output stream ptr to print to
 */
constexpr auto PrintTo(const Gains &gains, std::ostream *os) -> void {
  if (os == nullptr) return;

  *os << "Gains{";

  *os << ".p = ";
  PrintTo(gains.p, os);
  *os << ", ";

  *os << ".d = ";
  PrintTo(gains.d, os);
  *os << ", ";

  *os << "}";
}

constexpr auto operator<<(std::ostream &os, const Gains &gains)
    -> std::ostream & {
  PrintTo(gains, &os);
  return os;
}

// References ///////////////////////////////////////////////////////////////

/// Data structure to store pd controller references for the unit tests
struct References {
  using IdxType = Eigen::Index;

  Eigen::VectorXd tau;
  Eigen::VectorXd q;

  /**
   *  \brief Creates a References with random values in it
   *
   *  \param[in] tau_size Size of the tau vector needed
   *  \param[in] q_size Optional size of the q vector (default to tau_size)
   *
   *  \return return type
   */
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

/**
 *  \brief Prints \a ref to \a os
 *
 *  Use the following format:
 *  References{.tau = Vector{...}, .q = Vector{...}}
 *
 *  \param[in] refs The references we wish to print
 *  \param[inout] os The output stream ptr to print to
 */
constexpr auto PrintTo(const References &ref, std::ostream *os) -> void {
  if (os == nullptr) return;

  *os << "References{";

  *os << ".tau = ";
  PrintTo(ref.tau, os);
  *os << ", ";

  *os << ".q = ";
  PrintTo(ref.q, os);
  *os << ", ";

  *os << "}";
}

constexpr auto operator<<(std::ostream &os, const References &references)
    -> std::ostream & {
  PrintTo(references, &os);
  return os;
}

}  // namespace tests::utils

#endif  // LINEAR_FEEDBACK_CONTROLLER_TESTS__PD_CONTROLLER_HPP_
