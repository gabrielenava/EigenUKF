/**
 * @file SystemModel.cpp
 * @author Gabriele Nava
 * @date 2023
 */
#include <SystemModel.h>

// define the constructor. Populate the class variables
SystemModel::SystemModel(const Eigen::MatrixXd &_R, const Eigen::MatrixXd &_Q, double _dt)
    : R(_R), Q(_Q), dt(_dt) {}

SystemModel::~SystemModel() {}

Eigen::VectorXd SystemModel::dynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u, const double &dt)
{
  // return the state x(k+1) = f(x(k), u(k));
  Eigen::Matrix2d A(2, 2);
  Eigen::Vector2d B(2);
  Eigen::VectorXd x_dyn(2);

  // for the moment, implemented a simple kinematic model
  A << 1.0, dt, 0.0, 1.0;
  B << std::pow(dt, 2) / 2, dt;

  x_dyn = A * x + B * u;

  return x_dyn;
}

Eigen::VectorXd SystemModel::observationModel(const Eigen::VectorXd &x)
{
  // return the measurements vector
  static Eigen::MatrixXd h(1, 2);

  h << 1.0, 0.0;

  return h * x;
}
