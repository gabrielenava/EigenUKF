/**
 * @file UnscentedKF.h
 * @author Gabriele Nava
 * @date 2023
 */
#ifndef UNSCENTED_KF_H
#define UNSCENTED_KF_H

#include <random>
#include <memory>
#include <Eigen/Dense>
#include <SystemModel.h>

class UnscentedKF
{

public:
  // default constructor
  UnscentedKF(std::shared_ptr<SystemModel> _model, const Eigen::MatrixXd &_P,
              const Eigen::MatrixXd &_Q, const Eigen::MatrixXd &_R);

  // default destructor
  ~UnscentedKF();

  void init();
  void init(const Eigen::VectorXd &x);
  void predict(const Eigen::VectorXd &u, const double &dt);
  void update(const Eigen::VectorXd &y);

  Eigen::VectorXd get_state();
  Eigen::MatrixXd get_cov();

private:
  // internal variable and methods
  void ut(const Eigen::VectorXd &x, const Eigen::MatrixXd &P, std::vector<Eigen::VectorXd> &sigmaPt);

  std::vector<double> wc, ws; // weights.
  std::shared_ptr<SystemModel> model;

  Eigen::MatrixXd P;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd P0;    // initial P.
  Eigen::MatrixXd K;     // kalman gain.
  Eigen::MatrixXd I;     // unit matrix.
  Eigen::VectorXd x_hat; // estimated state.

  // internal parameters (not supposed to be edited)
  double alpha = 0.001;
  double beta = 2.0;
  double k = 0.0;

  // lambda parameter (to be updated by the constructor)
  double lambda = 0.0;
};

#endif /* end of include guard UNSCENTED_KF_H */
