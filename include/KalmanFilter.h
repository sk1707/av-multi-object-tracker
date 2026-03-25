#pragma once
#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(int state_dim, int meas_dim);

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

    Eigen::VectorXd getState() const { return x_; }

private:
    int state_dim_, meas_dim_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
};
