/**
 * @file SystemModel.h
 * @author Gabriele Nava
 * @date 2023
 */
#ifndef SYSTEM_MODEL_H
#define SYSTEM_MODEL_H

#include <Eigen/Dense>

class SystemModel
{

public:
  // default constructor
  SystemModel(
      const Eigen::MatrixXd &_R, // Measurement noise covariance.
      const Eigen::MatrixXd &_Q, // Process noise covariance.
      double _dt = 0.01);

  // default destructor
  ~SystemModel();

  Eigen::MatrixXd R, Q;

  // class methods
  Eigen::VectorXd dynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u, const double &dt);
  Eigen::VectorXd observationModel(const Eigen::VectorXd &x);

protected:
  // step time
  double dt;
};

#endif /* end of include guard SYSTEM_MODEL_H */
