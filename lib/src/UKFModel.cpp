/**
 * @file UKFModel.cpp
 * @author Gabriele Nava
 * @date 2024
 */
#include "UKFModel.h"

UKFModel::UKFModel()
{
}

UKFModel::UKFModel(const int nState, const int nMeas)
    : m_nState(nState)
    , m_nMeas(nMeas)
{
}

UKFModel::~UKFModel()
{
}

void UKFModel::updateProcessModel()
{
}

Eigen::VectorXd UKFModel::computePredictionFromModel(const Eigen::Ref<Eigen::VectorXd> x)
{
    return x;
}

Eigen::VectorXd UKFModel::computeObservationFromModel(const Eigen::Ref<Eigen::VectorXd> x)
{
    static Eigen::MatrixXd h(m_nMeas, m_nState);
    h.setZero();
    return h * x;
}
