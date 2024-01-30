/**
 * @file JetUKFModel.h
 * @author Gabriele Nava
 * @date 2024
 */
#ifndef JET_UKF_MODEL_H
#define JET_UKF_MODEL_H

#include "UKFModel.h"

/**
 * @class JetUKFModel
 * @brief Inherit and overwrite methods of the UKFModel class to implement
 * the prediction and observation models for jet thrust estimation.
 */
class JetUKFModel : public UKFModel
{
public:
    /**
     * @brief Construct a JetUKFModel object with inputs.
     * @param dt discretization time step.
     */
    JetUKFModel(const double& dt);

    /**
     * @brief Update the process model.
     * @return None.
     */
    void updateProcessModel(const Eigen::Ref<Eigen::VectorXd> u);

    /**
     * @brief Get the system state at next time instant using the prediction model.
     * @param x State at the current time instant;
     * @return State at the next time instant.
     */
    Eigen::VectorXd computePredictionFromModel(const Eigen::Ref<Eigen::VectorXd> x) override;

    /**
     * @brief Get the observed variables from the current state. The observed variables are
     * the ones to be compared with the available measurements.
     * @param x State at current time instant;
     * @return Observed variables.
     */
    Eigen::VectorXd computeObservationFromModel(const Eigen::Ref<Eigen::VectorXd> x) override;

    double m_dt;

    // state model parameters
    Eigen::Matrix2d m_A;
    Eigen::Vector2d m_B;
    Eigen::VectorXd m_u;

    // measurement model parameters
    Eigen::Vector2d m_h;
};

#endif /* end of include guard JET_UKF_MODEL_H */
