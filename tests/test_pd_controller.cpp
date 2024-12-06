#include "gtest/gtest.h"

#include <array>
#include <limits>
#include <optional>

#include "linear_feedback_controller/pd_controller.hpp"

using namespace linear_feedback_controller;

// Generate an array of a given type with size deduced by the number of arguments
template <typename ValueType, typename... Values>
constexpr auto MakeArray(Values &&... values) -> std::array<ValueType, sizeof...(Values)>
{
  return std::array<ValueType, sizeof...(Values)>{std::forward<Values>(values)...};
}

TEST(PdControllerTest, Ctor)
{
  EXPECT_NO_THROW({ const auto pd_ctrl = PDController(); });
}

struct Gains
{
  using IdxType = Eigen::VectorXd::Index;

  Eigen::VectorXd p;
  Eigen::VectorXd d;

  static auto Random(IdxType p_size, std::optional<IdxType> d_size = std::nullopt) -> Gains
  {
    return Gains{
      Eigen::VectorXd::Random(p_size),
      Eigen::VectorXd::Random(d_size.value_or(p_size)),
    };
  }
};

TEST(PdControllerTest, SetGains)
{
  auto pd_ctrl = PDController();

  {
    // FIXME: Is it valid ?
    SCOPED_TRACE("Size 0");
    EXPECT_NO_THROW({ pd_ctrl.set_gains(Eigen::VectorXd{}, Eigen::VectorXd{}); });
  }

  {
    SCOPED_TRACE("Same Sizes");
    for (const auto both_size : MakeArray<Gains::IdxType>(1, 3, 4, 50, 1000))
    {
      auto requested_gains = Gains::Random(both_size);
      EXPECT_NO_THROW({ pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
      // FIXME: How to test the validity ??

      // Test for special double values acceptance
      for (const auto special_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
           })
      {
        auto before = requested_gains.d(0);
        requested_gains.d(0) = special_value;
        EXPECT_NO_THROW({ pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
        requested_gains.d(0) = before;

        before = requested_gains.p(0);
        requested_gains.p(0) = special_value;
        EXPECT_NO_THROW({ pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
        requested_gains.p(0) = before;
      }
    }
  }

  {
    SCOPED_TRACE("Different Sizes");
    using SizeList = std::array<Gains::IdxType, 2>;
    for (const auto [p_size, d_size] :
         MakeArray<SizeList>(SizeList{1, 2}, SizeList{4, 3}, SizeList{50, 1000}))
    {
      const auto requested_gains = Gains::Random(p_size, d_size);
      // FIXME: I guess it should fail but set_gains doesn't provide any feedback on failure
      EXPECT_NO_THROW({ pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
    }
  }
}

struct References
{
  using IdxType = Eigen::VectorXd::Index;

  Eigen::VectorXd tau;
  Eigen::VectorXd q;

  static auto Random(IdxType tau_size, std::optional<IdxType> q_size = std::nullopt) -> References
  {
    return References{
      Eigen::VectorXd::Random(tau_size),
      Eigen::VectorXd::Random(q_size.value_or(tau_size)),
    };
  }
};

TEST(PdControllerTest, SetReferences)
{
  auto pd_ctrl = PDController();

  {
    // FIXME: Is it valid ?
    SCOPED_TRACE("Size 0");
    EXPECT_NO_THROW({ pd_ctrl.set_reference(Eigen::VectorXd{}, Eigen::VectorXd{}); });
  }

  {
    SCOPED_TRACE("Same Sizes");
    for (const auto both_size : MakeArray<References::IdxType>(1, 3, 4, 50, 1000))
    {
      auto requested_refs = References::Random(both_size);
      EXPECT_NO_THROW({ pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
      // FIXME: How to test the validity ??

      // Test for special double values acceptance
      for (const auto special_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
           })
      {
        auto before = requested_refs.tau(0);
        requested_refs.tau(0) = special_value;
        EXPECT_NO_THROW({ pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
        requested_refs.tau(0) = before;

        before = requested_refs.q(0);
        requested_refs.q(0) = special_value;
        EXPECT_NO_THROW({ pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
        requested_refs.q(0) = before;
      }
    }
  }

  {
    SCOPED_TRACE("Different Sizes");
    using SizeList = std::array<References::IdxType, 2>;
    for (const auto [tau_size, q_size] :
         MakeArray<SizeList>(SizeList{1, 2}, SizeList{4, 3}, SizeList{50, 1000}))
    {
      const auto requested_refs = References::Random(tau_size, q_size);
      // FIXME: I guess it should fail but set_ref doesn't provide any feedback on failure
      EXPECT_NO_THROW({ pd_ctrl.set_reference(requested_refs.tau, requested_refs.q); });
    }
  }
}
