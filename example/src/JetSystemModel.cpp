/**
 * @file JetSystemModel.cpp
 * @author Gabriele Nava
 * @date 2023
 */
#include <JetSystemModel.h>

// definition of the new constructor; initialize model parameters
JetSystemModel::JetSystemModel(int _nMeasures, double _dt) : nMeasures(_nMeasures), dt(_dt) {}
Eigen::Matrix2d JetSystemModel::A = Eigen::Matrix2d::Zero();
Eigen::Vector2d JetSystemModel::B = Eigen::Vector2d::Zero();
Eigen::Vector2d JetSystemModel::h = Eigen::Vector2d::Zero();

// definition of the child class methods
void JetSystemModel::configureModelParameters()
{
  // for jet engines, implemented a simple kinematic model
  this->A << 1.0, dt, 0.0, 1.0;
  this->B << std::pow(dt, 2) / 2, dt;

  // measurements are the observed thrusts
  this->h << 1.0, 0.0;
}

// overwrite the dynamicsModel class
Eigen::VectorXd JetSystemModel::dynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
  return (this->A * x + this->B * u);
}

// overwrite the observationModel class
Eigen::VectorXd JetSystemModel::observationModel(const Eigen::VectorXd &x)
{
  return this->h.transpose() * x;
}
