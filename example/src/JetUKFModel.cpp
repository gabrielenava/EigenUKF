/**
 * @file JetUKFModel.cpp
 * @author Gabriele Nava
 * @date 2024
 */
#include <JetUKFModel.h>

JetUKFModel::JetUKFModel(const double& dt)
    : m_dt(dt)
{
    // for jet engines, implemented a simple kinematic model
    m_A << 1.0, m_dt, 0.0, 1.0;
    m_B << std::pow(m_dt, 2) / 2, m_dt;

    // measurements are the observed thrusts
    m_h << 1.0, 0.0;
}

void JetUKFModel::updateProcessModel(const Eigen::Ref<Eigen::VectorXd> u)
{
    m_u = u;
}

Eigen::VectorXd JetUKFModel::computePredictionFromModel(const Eigen::Ref<Eigen::VectorXd> x)
{
    return (m_A * x + m_B * m_u);
}

Eigen::VectorXd JetUKFModel::computeObservationFromModel(const Eigen::Ref<Eigen::VectorXd> x)
{
    return m_h.transpose() * x;
}
