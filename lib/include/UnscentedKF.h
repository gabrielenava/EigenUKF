/**
 * @file UnscentedKF.h
 * @author Gabriele Nava
 * @date 2024
 */
#ifndef UNSCENTED_KF_H
#define UNSCENTED_KF_H

#include <Eigen/Dense>
#include <UKFModel.h>
#include <memory>
#include <vector>

/**
 * @class UnscentedKF
 * @brief Implementation of the Unscented Kalman Filter in c++. Provides methods to set up a routine
 * for estimation/filtering of user-defined quantities via UKF. Works alongside with the UKFModel
 * class which provides the prediction and observation models.
 */
class UnscentedKF
{
public:
    /**
     * @brief Construct an UnscentedKF object with no input variables.
     */
    UnscentedKF(){};

    /**
     * @brief Construct an UnscentedKF object with the specified variables.
     * @param model object of the SystemModel class;
     * @param P initial covariance matrix;
     * @param Q process noise covariance matrix;
     * @param R measurements noise covariance matrix.
     */
    UnscentedKF(std::shared_ptr<UKFModel> model,
                const Eigen::Ref<Eigen::MatrixXd> P,
                const Eigen::Ref<Eigen::MatrixXd> Q,
                const Eigen::Ref<Eigen::MatrixXd> R);

    /**
     * @brief Destruct the UnscentedKF object.
     */
    ~UnscentedKF();

    /**
     * @brief Initialize the UnscentedKF object. Initial state is set to zero.
     * @return None.
     */
    void init();

    /**
     * @brief Initialize the UnscentedKF object.
     * @param x initial state.
     * @return None.
     */
    void init(const Eigen::VectorXd& x);

    /**
     * @brief Predict the system state using the prediction model provided by UKFModel class.
     * @return None.
     */
    void predict();

    /**
     * @brief Update the state with the observation model provided by the UKMModel class
     * and corrections provided by real measurements.
     * @param y measurements vector
     * @return None.
     */
    void update(const Eigen::VectorXd& y);

    /**
     * @brief Get the current state estimated by the UKF.
     * @return the state estimated by the UKF.
     */
    Eigen::VectorXd getState();

    /**
     * @brief Get the current state covariance estimated by the UKF.
     * @return the covariance matrix estimated by the UKF.
     */
    Eigen::MatrixXd getCov();

private:
    // implementation of the Unscented Transform
    void
    ut(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, std::vector<Eigen::VectorXd>& sigmaPt);

    std::shared_ptr<UKFModel> m_model; // pointer to the prediction and measurements models

    std::vector<double> m_wc, m_ws; // weights

    Eigen::MatrixXd m_P; // state covariance
    Eigen::MatrixXd m_Q; // process noise covariance
    Eigen::MatrixXd m_R; // measurements noise covariance
    Eigen::MatrixXd m_P0; // initial P
    Eigen::MatrixXd m_K; // kalman gain
    Eigen::MatrixXd m_I; // unit matrix
    Eigen::VectorXd m_x_hat; // estimated state

    // internal parameters (not supposed to be edited)
    double m_alpha = 0.001;
    double m_beta = 2.0;
    double m_k = 0.0;
    double m_lambda = 0.0;
};

#endif /* end of include guard UNSCENTED_KF_H */
