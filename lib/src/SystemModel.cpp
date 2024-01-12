/**
 * @file SystemModel.cpp
 * @author Gabriele Nava
 * @date 2023
 */
#include <SystemModel.h>

// define the constructors and destructor
SystemModel::SystemModel() {}
SystemModel::SystemModel(int _nMeasures) : nMeasures(_nMeasures) {}
SystemModel::~SystemModel() {}

Eigen::VectorXd SystemModel::dynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
  // default behaviour: return a vector of zeros, of the same length of x
  Eigen::VectorXd x_upd(x.size());
  x_upd.setZero();
  return x_upd;
}

Eigen::VectorXd SystemModel::observationModel(const Eigen::VectorXd &x)
{
  // default behaviour: return a vector of zeros, of the same length of x
  Eigen::VectorXd y(this->nMeasures);
  y.setZero();
  return y;
}
