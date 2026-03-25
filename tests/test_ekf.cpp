#include <gtest/gtest.h>
#include "ekf.h"

TEST(EKFTest, InitializesCorrectly) {
    EKF ekf(4, 2);
    Eigen::VectorXd x0(4); x0 << 1, 2, 0.5, 0.3;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd Q  = Eigen::MatrixXd::Identity(4, 4) * 0.1;
    Eigen::MatrixXd R  = Eigen::MatrixXd::Identity(2, 2) * 0.3;
    ekf.init(x0, P0, Q, R);
    EXPECT_NEAR(ekf.getState()(0), 1.0, 1e-6);
    EXPECT_NEAR(ekf.getState()(1), 2.0, 1e-6);
}

TEST(EKFTest, PredictMovesState) {
    EKF ekf(4, 2);
    Eigen::VectorXd x0(4); x0 << 0, 0, 1, 0;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd Q  = Eigen::MatrixXd::Identity(4, 4) * 0.1;
    Eigen::MatrixXd R  = Eigen::MatrixXd::Identity(2, 2) * 0.3;
    ekf.init(x0, P0, Q, R);
    ekf.predict(0.1);
    EXPECT_NEAR(ekf.getState()(0), 0.1, 1e-5);
}

TEST(EKFTest, UpdateReducesUncertainty) {
    EKF ekf(4, 2);
    Eigen::VectorXd x0(4); x0 << 5, 5, 1, 0;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4) * 10;
    Eigen::MatrixXd Q  = Eigen::MatrixXd::Identity(4, 4) * 0.1;
    Eigen::MatrixXd R  = Eigen::MatrixXd::Identity(2, 2) * 0.3;
    ekf.init(x0, P0, Q, R);
    double before = ekf.getCovariance().trace();
    Eigen::VectorXd z(2); z << 5.1, 5.0;
    ekf.update(z, "lidar");
    double after = ekf.getCovariance().trace();
    EXPECT_LT(after, before);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
