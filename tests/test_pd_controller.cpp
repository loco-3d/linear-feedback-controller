#include <array>
#include <limits>
#include <optional>

#include "gtest/gtest.h"
#include "linear_feedback_controller/pd_controller.hpp"

using namespace linear_feedback_controller;

// Generate an array of a given type with its size deduced by the number of args
//
// This may be usefull when you don't want to specify by hand the number of
// elements inside an array but want to force the value_type of the array (in
// order to force conversion & co ...)
template <typename ValueType, typename... T>
constexpr auto MakeArray(T &&...values) -> std::array<ValueType, sizeof...(T)> {
  return std::array<ValueType, sizeof...(T)>{std::forward<T>(values)...};
}

// RAII mutator, use to temporary modify a value given a reference and a tmp
// value
//
// WARNING: Dangling references & co ...
template <typename ValueType>
struct TemporaryMutate {
  TemporaryMutate() = delete;

  template <typename T>
  TemporaryMutate(ValueType &data, T &&tmp_value)
      : data_(data), before_(data_) {
    data_ = std::forward<T>(tmp_value);
  }

  virtual ~TemporaryMutate() { data_ = before_; }

  constexpr auto GetOldValue() const -> const ValueType & { return before_; }

 private:
  ValueType &data_;
  ValueType before_;
};

TEST(PdControllerTest, Ctor) {
  EXPECT_NO_THROW({ const auto pd_ctrl = PDController(); });
}

struct Gains {
  using IdxType = Eigen::Index;

  Eigen::VectorXd p;
  Eigen::VectorXd d;

  static auto Random(IdxType p_size,
                     std::optional<IdxType> d_size = std::nullopt) -> Gains {
    return Gains{
        .p = Eigen::VectorXd::Random(p_size),
        .d = Eigen::VectorXd::Random(d_size.value_or(p_size)),
    };
  }
};

TEST(PdControllerTest, SetGains) {
  auto pd_ctrl = PDController();

  EXPECT_NO_THROW({ pd_ctrl.set_gains(Eigen::VectorXd{}, Eigen::VectorXd{}); });

  for (const auto size : MakeArray<Gains::IdxType>(1, 3, 4, 50, 1000)) {
    SCOPED_TRACE(::testing::Message() << "Same gains size: " << size);
    auto requested_gains = Gains::Random(size);
    EXPECT_NO_THROW(
        { pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
    // FIXME: How to test the validity ??

    // Test for special double values acceptance
    for (const auto special_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
         }) {
      SCOPED_TRACE(::testing::Message() << "Special value: " << special_value);

      {
        SCOPED_TRACE("P");
        auto _ = TemporaryMutate{requested_gains.p(0), special_value};
        EXPECT_NO_THROW(
            { pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
      }

      {
        SCOPED_TRACE("D");
        auto _ = TemporaryMutate{requested_gains.d(0), special_value};
        EXPECT_NO_THROW(
            { pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
      }
    }
  }

  using SizePair = std::array<Gains::IdxType, 2>;
  for (const auto [p_size, d_size] : MakeArray<SizePair>(
           SizePair{1, 2}, SizePair{4, 3}, SizePair{50, 1000})) {
    SCOPED_TRACE(::testing::Message()
                 << "Different Sizes: P = " << p_size << ", D = " << d_size);
    const auto requested_gains = Gains::Random(p_size, d_size);
    // FIXME: I guess it should fail but set_gains doesn't provide any feedback
    // on failure
    EXPECT_NO_THROW(
        { pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
  }
}

struct References {
  using IdxType = Eigen::Index;

  Eigen::VectorXd tau;
  Eigen::VectorXd q;

  static auto Random(IdxType tau_size, std::optional<IdxType> q_size =
                                           std::nullopt) -> References {
    return References{
        .tau = Eigen::VectorXd::Random(tau_size),
        .q = Eigen::VectorXd::Random(q_size.value_or(tau_size)),
    };
  }
};

TEST(PdControllerTest, SetReferences) {
  auto pd_ctrl = PDController();

  EXPECT_NO_THROW(
      { pd_ctrl.set_reference(Eigen::VectorXd{}, Eigen::VectorXd{}); });

  for (const auto size : MakeArray<References::IdxType>(1, 3, 4, 50, 1000)) {
    SCOPED_TRACE(::testing::Message() << "Same gains size: " << size);

    auto requested_refs = References::Random(size);
    EXPECT_NO_THROW(
        { pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
    // FIXME: How to test the validity ??

    // Test for special double values acceptance
    for (const auto special_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
         }) {
      SCOPED_TRACE(::testing::Message() << "Special value: " << special_value);

      {
        SCOPED_TRACE("TAU");
        auto _ = TemporaryMutate{requested_refs.tau(0), special_value};
        EXPECT_NO_THROW(
            { pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
      }

      {
        SCOPED_TRACE("Q");
        auto _ = TemporaryMutate{requested_refs.q(0), special_value};
        EXPECT_NO_THROW(
            { pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
      }
    }
  }

  using SizePair = std::array<References::IdxType, 2>;
  for (const auto [tau_size, q_size] : MakeArray<SizePair>(
           SizePair{1, 2}, SizePair{4, 3}, SizePair{50, 1000})) {
    SCOPED_TRACE(::testing::Message() << "Different Sizes: TAU = " << tau_size
                                      << ", Q = " << q_size);
    const auto requested_refs = References::Random(tau_size, q_size);
    // FIXME: I guess it should fail but set_ref doesn't provide any feedback on
    // failure
    EXPECT_NO_THROW(
        { pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
  }
}

TEST(PdControllerTest, ComputeControl) {
  auto pd_ctrl = PDController();

  for (const auto size : {1u, 2u, 5u, 20u}) {
    SCOPED_TRACE(::testing::Message() << "Problem size = " << size);

    const auto gains = Gains::Random(size);
    pd_ctrl.set_gains(gains.p, gains.d);

    const auto refs = References::Random(size);
    pd_ctrl.set_reference(refs.tau, refs.q);

    const Eigen::VectorXd arg_q = Eigen::VectorXd::Random(size);
    const Eigen::VectorXd arg_v = Eigen::VectorXd::Random(size);

    // o = tau_r - (p * (q - q_r)) - (d * v)

    // clang-format off
  const Eigen::VectorXd expected_control =
    (refs.tau.array()
     - (gains.p.array() * (arg_q - refs.q).array())
     - (gains.d.array() * arg_v.array()));
    // clang-format on

    EXPECT_EQ(pd_ctrl.compute_control(arg_q, arg_v), expected_control);
  }

  // TODO test wrong sizes & co
}
