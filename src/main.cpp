#include <iostream>
#include "KalmanFilter.h"

int main() {
    int state_dim = 2;
    int meas_dim = 1;

    KalmanFilter kf(state_dim, meas_dim);

    Eigen::VectorXd x0(state_dim);
    x0 << 0, 0;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(state_dim, state_dim);
    kf.init(x0, P0);

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim, state_dim);
    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(state_dim, state_dim);
    Eigen::VectorXd z(meas_dim);
    z << 1.0;
    Eigen::MatrixXd H(meas_dim, state_dim);
    H << 1, 0;
    Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(meas_dim, meas_dim);

    kf.predict(F, Q);
    kf.update(z, H, R);

    std::cout << "Updated state: " << kf.getState().transpose() << "\n";
    return 0;
}
