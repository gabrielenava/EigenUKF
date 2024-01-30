/**
 * @file UnscentedKF.cpp
 * @author Gabriele Nava
 * @date 2024
 */
#include <UnscentedKF.h>

UnscentedKF::UnscentedKF(std::shared_ptr<UKFModel> model,
                         const Eigen::Ref<Eigen::MatrixXd> P,
                         const Eigen::Ref<Eigen::MatrixXd> Q,
                         const Eigen::Ref<Eigen::MatrixXd> R)
    : m_model(model)
    , m_P(P)
    , m_Q(Q)
    , m_R(R)
    , m_P0(P)
    , m_x_hat(P.rows())
{
    // calculate lambda parameter
    m_lambda = std::pow(m_alpha, 2) * (m_P.rows() + m_k) - m_P.rows();

    // reserve memory for weights vectors
    m_wc.reserve(2 * m_P.rows() + 1);
    m_ws.reserve(2 * m_P.rows() + 1);

    // populate weights vectors
    m_ws.emplace_back(m_lambda / (m_P.rows() + m_lambda));
    m_wc.emplace_back(m_ws[0] + (1.0 - std::pow(m_alpha, 2) + m_beta));

    for (size_t i = 1; i < 2 * m_P.rows() + 1; i++)
    {
        double val = 1.0 / (2.0 * (m_lambda + m_P.rows()));
        m_ws.emplace_back(val);
        m_wc.emplace_back(val);
    }
}

UnscentedKF::~UnscentedKF()
{
}

void UnscentedKF::init()
{
    // set initial values. Initial state set to zero
    m_P = m_P0;
    m_x_hat.setZero();
    m_I = Eigen::MatrixXd::Identity(m_P.rows(), m_P.rows());
}

void UnscentedKF::init(const Eigen::VectorXd& x)
{
    // set initial values. Initial state passed by the user
    init();
    m_x_hat = x;
}

void UnscentedKF::ut(const Eigen::VectorXd& x,
                     const Eigen::MatrixXd& P,
                     std::vector<Eigen::VectorXd>& sigmaPt)
{
    // Calculate square root of P, i.e, P = L*L^t
    Eigen::LLT<Eigen::MatrixXd> chol(P);
    Eigen::MatrixXd L = chol.matrixL();

    sigmaPt.clear();
    sigmaPt.reserve(2 * P.rows() + 1);

    // initialize sigma points with current estimate
    sigmaPt.emplace_back(m_x_hat);

    for (size_t i = 0; i < P.rows(); i++)
    {
        // generate sigma points
        Eigen::VectorXd preSigmaPt1 = x + (sqrt(P.rows() + m_lambda) * L).col(i);
        Eigen::VectorXd preSigmaPt2 = x - (sqrt(P.rows() + m_lambda) * L).col(i);
        sigmaPt.emplace_back(preSigmaPt1);
        sigmaPt.emplace_back(preSigmaPt2);
    }
}

void UnscentedKF::predict()
{
    // use the model to predict the value of the state at (k+1)

    // generate sigma points using unscented transform (ut)
    std::vector<Eigen::VectorXd> sigmaPt;
    ut(m_x_hat, m_P, sigmaPt);

    // predict the new state using the dynamic model
    Eigen::VectorXd x_new(m_x_hat.rows());
    x_new.setZero();

    for (size_t i = 0; i < 2 * m_P.rows() + 1; i++)
    {
        // propagate the sigma points using the dynamics
        sigmaPt[i] = m_model->computePredictionFromModel(sigmaPt[i]);
        x_new += m_ws[i] * sigmaPt[i];
    }

    // update the covariance matrix
    Eigen::MatrixXd P_new(m_P.rows(), m_P.rows());
    P_new.setZero();

    for (size_t i = 0; i < 2 * m_P.rows() + 1; i++)
    {
        P_new += m_wc[i] * (sigmaPt[i] - x_new) * (sigmaPt[i] - x_new).transpose();
    }

    m_x_hat = x_new;
    m_P = P_new + m_Q;
}

void UnscentedKF::update(const Eigen::VectorXd& y)
{
    // use the measurements and observation model to correct the prediction

    // generate sigma points using unscented transform (ut)
    std::vector<Eigen::VectorXd> sigmaPt;
    ut(m_x_hat, m_P, sigmaPt);

    std::vector<Eigen::VectorXd> gamma;
    gamma.resize(sigmaPt.size());

    // compute the observed quantities via observation model
    Eigen::VectorXd z(y.rows());
    z.setZero();

    for (size_t i = 0; i < 2 * m_P.rows() + 1; i++)
    {
        gamma[i] = m_model->computeObservationFromModel(sigmaPt[i]);
        z += m_ws[i] * gamma[i];
    }

    // update the covariance matrices
    Eigen::MatrixXd P_obs(z.rows(), z.rows()), P_est(m_P.rows(), z.rows());
    P_obs.setZero();
    P_est.setZero();

    for (size_t i = 0; i < 2 * m_P.rows() + 1; i++)
    {
        P_obs += m_wc[i] * (gamma[i] - z) * (gamma[i] - z).transpose();
        P_est += m_wc[i] * (sigmaPt[i] - m_x_hat) * (gamma[i] - z).transpose();
    }

    // compute the Kalman gain and estimate the state and covariance
    m_K = P_est * (P_obs + m_R).inverse();
    m_x_hat += m_K * (y - z);
    m_P = m_P - m_K * (P_obs + m_R) * m_K.transpose();
}

Eigen::VectorXd UnscentedKF::getState()
{
    return m_x_hat;
}

Eigen::MatrixXd UnscentedKF::getCov()
{
    return m_P;
}
