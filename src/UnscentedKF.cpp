/**
 * @file UnscentedKF.cpp
 * @author Gabriele Nava
 * @date 2023
 */
#include <UnscentedKF.h>

UnscentedKF::UnscentedKF(std::shared_ptr<SystemModel> _model, const Eigen::MatrixXd &_P)
    : model(_model), P(_P), P0(_P), x_hat(_P.rows())
{
  // calculate lambda parameter
  lambda = std::pow(alpha, 2) * (P.rows() + k) - P.rows();

  // reserve memory for weights vectors
  wc.reserve(2 * P.rows() + 1);
  ws.reserve(2 * P.rows() + 1);

  // populate weights vectors
  ws.push_back(lambda / (P.rows() + lambda));
  wc.push_back(ws[0] + (1.0 - std::pow(alpha, 2) + beta));

  for (size_t i = 1; i < 2 * P.rows() + 1; i++)
  {
    double val = 1.0 / (2.0 * (lambda + P.rows()));
    ws.push_back(val);
    wc.push_back(val);
  }
}

UnscentedKF::~UnscentedKF() {}

void UnscentedKF::init()
{
  // set initial values. Initial state set to zero
  P = P0;
  x_hat.setZero();
  I = Eigen::MatrixXd::Identity(P.rows(), P.rows());
}

void UnscentedKF::init(const Eigen::VectorXd &x)
{
  // set initial values. Initial state passed by the user
  init();
  x_hat = x;
}

void UnscentedKF::ut(const Eigen::VectorXd &x, const Eigen::MatrixXd &P, std::vector<Eigen::VectorXd> &sigmaPt)
{
  // Calculate square root of P, i.e, P = L*L^t
  Eigen::LLT<Eigen::MatrixXd> chol(P);
  Eigen::MatrixXd L = chol.matrixL();

  sigmaPt.clear();
  sigmaPt.reserve(2 * P.rows() + 1);

  // initialize sigma points with current estimate
  sigmaPt.push_back(x_hat);

  for (size_t i = 0; i < P.rows(); i++)
  {
    // generate sigma points
    Eigen::VectorXd preSigmaPt1 = x + (sqrt(P.rows() + lambda) * L).col(i);
    Eigen::VectorXd preSigmaPt2 = x - (sqrt(P.rows() + lambda) * L).col(i);
    sigmaPt.push_back(preSigmaPt1);
    sigmaPt.push_back(preSigmaPt2);
  }
}

void UnscentedKF::predict(const Eigen::VectorXd &u, const double &dt)
{
  // use the model to predict the value of the state at (k+1)

  // generate sigma points using unscented transform (ut)
  std::vector<Eigen::VectorXd> sigmaPt;
  ut(x_hat, P, sigmaPt);

  // predict the new state using the dynamic model
  Eigen::VectorXd x_new(x_hat.rows());
  x_new.setZero();

  for (size_t i = 0; i < 2 * P.rows() + 1; i++)
  {
    // propagate the sigma points using the dynamics
    sigmaPt[i] = model->dynamicsModel(sigmaPt[i], u, dt);
    x_new += ws[i] * sigmaPt[i];
  }

  // update the covariance matrix
  Eigen::MatrixXd P_new(P.rows(), P.rows());
  P_new.setZero();

  for (size_t i = 0; i < 2 * P.rows() + 1; i++)
  {
    P_new += wc[i] * (sigmaPt[i] - x_new) * (sigmaPt[i] - x_new).transpose();
  }

  x_hat = x_new;
  P = P_new + model->Q;
}

void UnscentedKF::update(const Eigen::VectorXd &y)
{
  // use the measurements to correct the prediction

  // generate sigma points using unscented transform (ut)
  std::vector<Eigen::VectorXd> sigmaPt;
  ut(x_hat, P, sigmaPt);

  std::vector<Eigen::VectorXd> gamma;
  gamma.resize(sigmaPt.size());

  // compute the observed quantities via observation model
  Eigen::VectorXd z(y.rows());
  z.setZero();

  for (size_t i = 0; i < 2 * P.rows() + 1; i++)
  {
    gamma[i] = model->observationModel(sigmaPt[i]);
    z += ws[i] * gamma[i];
  }

  // update the covariance matrices
  Eigen::MatrixXd P_obs(z.rows(), z.rows()), P_est(P.rows(), z.rows());
  P_obs.setZero();
  P_est.setZero();

  for (size_t i = 0; i < 2 * P.rows() + 1; i++)
  {
    P_obs += wc[i] * (gamma[i] - z) * (gamma[i] - z).transpose();
    P_est += wc[i] * (sigmaPt[i] - x_hat) * (gamma[i] - z).transpose();
  }

  // compute the Kalman gain and estimate the state and covariance
  K = P_est * (P_obs + model->R).inverse();
  x_hat += K * (y - z);
  P = P - K * (P_obs + model->R) * K.transpose();
}

Eigen::VectorXd UnscentedKF::get_state()
{
  return x_hat;
}

Eigen::MatrixXd UnscentedKF::get_cov()
{
  return P;
}
