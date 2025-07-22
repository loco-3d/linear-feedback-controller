#include "linear_feedback_controller/lf_controller.hpp"
#include "utils/mock_robot_model_builder_smart.hpp"
#include "utils/mock_robot_model_builder.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace linear_feedback_controller;

// basic fixture "happy path"
class LFControllerTest : public ::testing::Test {
protected:
    std::shared_ptr<SmartMockRobotModelBuilder> mock_robot_builder_;
    std::unique_ptr<LFController> controller_;

    void SetUp() override {
        mock_robot_builder_ = std::make_shared<SmartMockRobotModelBuilder>();
        controller_ = std::make_unique<LFController>();
        controller_->initialize(mock_robot_builder_);
    }
};

TEST_F(LFControllerTest, CanBeConstructed) {
    ASSERT_NE(controller_, nullptr);
    ASSERT_NE(mock_robot_builder_, nullptr);
}

TEST_F(LFControllerTest, ComputesCorrectControlSignal) {
    // --- 1. TEST INPUTS CONFIGURATION ---
    // we use the robotModel inside the smart mocked class (nv = nq = 2)
    int nq = 2;
    int nv = 2;

    // We define measured and desired states
    Eigen::VectorXd q_measured(nq); q_measured << 0.1, -0.1;
    Eigen::VectorXd v_measured(nv); v_measured << 0.2, 0.0;
    
    Eigen::VectorXd q_desired(nq); q_desired << 0.0, 0.0;
    Eigen::VectorXd v_desired(nv); v_desired << 0.0, 0.0;
    
    // We define feedforward and gains (these are supposed to be an output of crocoddyl)
    Eigen::VectorXd feedforward_input(nv);
    feedforward_input << 9.5, 1.4;
    Eigen::MatrixXd feedback_gain_input(nv, 2 * nv);
    // K = [Kp | Kd]
    // Kp = diag(100, 120)
    // Kd = diag(20, 25)
    feedback_gain_input << 100,   0, 20,  0,
                               0, 120,  0, 25;

    // We put all the generated data on expected format (ROS messages)
    linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
    sensor_msg.joint_state.position = q_measured;
    sensor_msg.joint_state.velocity = v_measured;

    linear_feedback_controller_msgs::Eigen::Control control_msg;
    control_msg.initial_state.joint_state.position = q_desired;
    control_msg.initial_state.joint_state.velocity = v_desired;
    control_msg.feedforward = feedforward_input;
    control_msg.feedback_gain = feedback_gain_input;


    // --- 2. WE MANUALY CALCULATE EXPECTED RESULT ---

    // Compute error state
    Eigen::VectorXd diff_state_expected(2 * nv);
    // Error on position is not a simple soustraction
    pinocchio::difference(mock_robot_builder_->get_model(), q_measured, q_desired,
                          diff_state_expected.head(nv));

    // Error on speed is more simple
    diff_state_expected.tail(nv) = v_desired - v_measured;

    // Compute command law
    // τ = τ_ff + K * x_err
    Eigen::VectorXd expected_control(nv);
    expected_control = feedforward_input + feedback_gain_input * diff_state_expected;


    // --- 3. WE CALL THE FUNCTION TO BE TESTED ---

    // First we need to precise how the mock function should react
    EXPECT_CALL(*mock_robot_builder_, construct_robot_state(testing::_, testing::_, testing::_))
    .WillOnce(testing::DoAll(
        testing::SetArgReferee<1>(q_desired),
        testing::SetArgReferee<2>(v_desired)
    ))
    .WillOnce(testing::DoAll(
        testing::SetArgReferee<1>(q_measured),
        testing::SetArgReferee<2>(v_measured)
    ));
    
    const Eigen::VectorXd& actual_control = controller_->compute_control(sensor_msg, control_msg);


    // --- 4. VERIFICATIONS ---

    // Debug
    std::cout << "Actual control:   " << actual_control.transpose() << std::endl;
    std::cout << "Expected control: " << expected_control.transpose() << std::endl;

    ASSERT_TRUE(actual_control.isApprox(expected_control, 1e-9));
}


// Robustness fixture
class LFControllerRobustnessTest : public ::testing::Test {
protected:
    std::shared_ptr<MockRobotModelBuilder> mock_robot_builder_;
    std::unique_ptr<LFController> controller_;

    void SetUp() override {
        controller_ = std::make_unique<LFController>();
    }

};

// initialisation with nullptr model
TEST_F(LFControllerRobustnessTest, InitializeWithNullModelThrows) {
    EXPECT_THROW(controller_->initialize(nullptr), std::invalid_argument);
}

// call of compute_control on a non-initialized controler 
TEST_F(LFControllerRobustnessTest, ComputeControlThrowsIfNotInitialized) {
    linear_feedback_controller_msgs::Eigen::Sensor sensor_msg;
    linear_feedback_controller_msgs::Eigen::Control control_msg;

    EXPECT_THROW(controller_->compute_control(sensor_msg, control_msg), std::runtime_error);
}

// TEST_F(LFControllerRobustnessTest, ThrowsOnMismatchedMessageSizes) {
//     // TODO: Implémentez ce test.
//     mock_robot_builder_ = std::make_shared<SmartMockRobotModelBuilder>();
//     controller_->initialize(mock_robot_builder_);
//     // 1. Créez un contrôleur et initialisez-le avec un mock qui attend un DoF de 6.
//     // 2. Créez un `control_msg` avec `q_ref` de taille 7 (invalide).
//     // 3. Appelez `compute_control`.
//     // 4. Utilisez EXPECT_THROW(..., std::invalid_argument) pour vérifier qu'une erreur est levée.
// }

// TEST_F(LFControllerRobustnessTest, HandlesSpecialFloatValuesInMessages) {
//     // TODO: Implémentez ce test.
//     // 1. Créez et initialisez le contrôleur.
//     // 2. Créez un `sensor_msg` et mettez une valeur NaN ou Infinity dans `v`.
//     // 3. Appelez `compute_control`.
//     // 4. Si le contrôleur doit vérifier cela, utilisez EXPECT_THROW. Sinon,
//     //    ce test peut servir à documenter le comportement actuel.
// }
