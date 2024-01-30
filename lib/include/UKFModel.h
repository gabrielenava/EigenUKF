/**
 * @file UKFModel.h
 * @author Gabriele Nava
 * @date 2024
 */
#ifndef UKF_MODEL_H
#define UKF_MODEL_H

#include <Eigen/Dense>

/**
 * @class UKFModel
 * @brief Define the prediction and observation models required by the UnscentedKF class.
 * This is an abstract class: the user must create his own child class derived from UKFModel,
 * and implement with it the prediction and observation models for his specific problem.
 */
class UKFModel
{
public:
    /**
     * @brief Construct an UKFModel object with no inputs.
     */
    UKFModel();

    /**
     * @brief Construct an UKFModel object with inputs.
     * @param nState size of state  vector;
     * @param nMeas size of measurement vector.
     */
    UKFModel(const int nState, const int nMeas);

    /**
     * @brief Destruct the UKFModel object.
     */
    ~UKFModel();

protected:
    /**
     * @brief Update the process model. Virtual function to be overloaded by the user's child class.
     * @return None.
     */
    virtual void updateProcessModel();

    /**
     * @brief Get the system state at next time instant using the prediction model.
     * Virtual function to be overloaded by the user's child class.
     * @param x State at the current time instant;
     * @return State at the next time instant.
     */
    virtual Eigen::VectorXd computePredictionFromModel(const Eigen::Ref<Eigen::VectorXd> x);

    /**
     * @brief Get the observed variables from the current state. The observed variables are
     * the ones to be compared with the available measurements.
     * Virtual function to be overloaded by the user's child class.
     * @param x State at current time instant;
     * @return Observed variables.
     */
    virtual Eigen::VectorXd computeObservationFromModel(const Eigen::Ref<Eigen::VectorXd> x);

    int m_nState = 1;
    int m_nMeas = 1;

    friend class UnscentedKF;
};

#endif /* end of include guard UKF_MODEL_H */
