#include <limits>  // numeric_limits

#include "utils/eigen.hpp"
#include "utils/mutation.hpp"
using tests::utils::TemporaryMutate;

#include "utils/pd_controller.hpp"
using tests::utils::Gains;
using tests::utils::References;

#include "linear_feedback_controller/pd_controller.hpp"
using linear_feedback_controller::PDController;

#include "gtest/gtest.h"

TEST(PdControllerTest, Ctor) {
  EXPECT_NO_THROW({ const auto pd_ctrl = PDController(); });
}

TEST(PdControllerTest, DISABLED_SetGainsEmpty) {
  auto pd_ctrl = PDController();

  EXPECT_ANY_THROW(
      { pd_ctrl.set_gains(Eigen::VectorXd{}, Eigen::VectorXd{}); });
}

TEST(PdControllerTest, DISABLED_SetGainsWithSpecialDouble) {
  auto pd_ctrl = PDController();

  constexpr auto size = 3u;
  auto requested_gains = Gains::Random(size);

  // Test for special double values acceptance or not ?
  for (const auto &ref : {
           std::ref(requested_gains.d(0)),
           // std::ref(requested_gains.d(size - 1)),

           std::ref(requested_gains.p(0)),
           // std::ref(requested_gains.p(size - 1)),
       }) {
    for (auto tmp_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
         }) {
      auto mutation = TemporaryMutate(ref.get(), tmp_value);
      SCOPED_TRACE(::testing::Message() << '\n' << requested_gains);

      // TODO: Is it an error or not ?
      EXPECT_ANY_THROW(
          { pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
    }
  }
}

TEST(PdControllerTest, DISABLED_SetGainsWithDifferentSizes) {
  auto pd_ctrl = PDController();

  for (auto &&requested_gains :
       {
           Gains::Random(1, 2),
           Gains::Random(4, 3),
           Gains::Random(50, 1000),
       })

  {
    SCOPED_TRACE(::testing::Message() << '\n' << requested_gains);
    EXPECT_ANY_THROW(
        { pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
  }
}

TEST(PdControllerTest, SetGains) {
  auto pd_ctrl = PDController();

  for (auto &&requested_gains :
       {
           Gains::Random(1),
           Gains::Random(3),
           Gains::Random(4),
           Gains::Random(50),
           Gains::Random(1000),
       })

  {
    SCOPED_TRACE(::testing::Message() << '\n' << requested_gains);
    EXPECT_NO_THROW(
        { pd_ctrl.set_gains(requested_gains.p, requested_gains.d); });
    // FIXME: How to confirm that it succeeded ?
  }
}

TEST(PdControllerTest, DISABLED_SetReferencesWithSpecialDouble) {
  auto pd_ctrl = PDController();

  constexpr auto size = 3u;
  auto requested_references = References::Random(size);

  // Test for special double values acceptance or not ?
  for (const auto &ref : {
           std::ref(requested_references.tau(0)),
           // std::ref(requested_references.tau(size - 1)),

           std::ref(requested_references.q(0)),
           // std::ref(requested_references.q(size - 1)),
       }) {
    for (auto tmp_value : {
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::signaling_NaN(),
         }) {
      auto mutation = TemporaryMutate(ref.get(), tmp_value);
      SCOPED_TRACE(::testing::Message() << '\n' << requested_references);

      // TODO: Is it an error or not ?
      EXPECT_ANY_THROW({
        pd_ctrl.set_reference(requested_references.tau, requested_references.q);
      });
    }
  }
}

TEST(PdControllerTest, DISABLED_SetReferencesWithDifferentSizes) {
  auto pd_ctrl = PDController();

  for (auto &&requested_references :
       {
           References::Random(1, 2),
           References::Random(4, 3),
           References::Random(50, 1000),
       })

  {
    SCOPED_TRACE(::testing::Message() << '\n' << requested_references);
    EXPECT_ANY_THROW({
      pd_ctrl.set_reference(requested_references.tau, requested_references.q);
    });
  }
}

TEST(PdControllerTest, SetReferences) {
  auto pd_ctrl = PDController();

  for (auto &&requested_references :
       {
           References::Random(1),
           References::Random(3),
           References::Random(4),
           References::Random(50),
           References::Random(1000),
       })

  {
    SCOPED_TRACE(::testing::Message() << '\n' << requested_references);
    EXPECT_NO_THROW({
      pd_ctrl.set_reference(requested_references.tau, requested_references.q);
    });
    // FIXME: How to confirm that it succeeded ?
  }
}

TEST(PdControllerTest, ComputeControl) {
  auto pd_ctrl = PDController();

  for (const auto size : {1u, 2u, 5u, 20u}) {
    const auto gains = Gains::Random(size);
    const auto refs = References::Random(size);

    const Eigen::VectorXd arg_q = Eigen::VectorXd::Random(size);
    const Eigen::VectorXd arg_v = Eigen::VectorXd::Random(size);

    std::stringstream trace_log;
    trace_log << '\n' << gains << '\n' << refs;

    trace_log << "\nQ = ";
    PrintTo(arg_q, &trace_log);

    trace_log << "\nV = ";
    PrintTo(arg_v, &trace_log);

    SCOPED_TRACE(trace_log.str());

    // clang-format off
    // o = tau_r - (p * (q - q_r)) - (d * v)
    const Eigen::VectorXd expected_control =
      (refs.tau.array()
       - (gains.p.array() * (arg_q - refs.q).array())
       - (gains.d.array() * arg_v.array()));
    // clang-format on

    SetTo(pd_ctrl, gains);
    SetTo(pd_ctrl, refs);
    EXPECT_EQ(pd_ctrl.compute_control(arg_q, arg_v), expected_control);

    // NOTE: compute_control use asserts for errors handling
    EXPECT_DEBUG_DEATH(
        {
          const auto _ = pd_ctrl.compute_control(
              arg_q, Eigen::VectorXd::Random(arg_q.size() + 1));
        },
        ::testing::ContainsRegex("Size missmatch"));

    EXPECT_DEBUG_DEATH(
        {
          const auto _ = pd_ctrl.compute_control(
              Eigen::VectorXd::Random(arg_v.size() + 1), arg_v);
        },
        ::testing::ContainsRegex("Size missmatch"));

    EXPECT_DEBUG_DEATH(
        {
          const auto _ = pd_ctrl.compute_control(
              Eigen::VectorXd::Random(refs.q.size() + 1),
              Eigen::VectorXd::Random(refs.q.size() + 1));
        },
        ::testing::ContainsRegex("Size missmatch"));
  }
}
