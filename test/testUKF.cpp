/**
 * @file testUKF.cpp
 * @author Gabriele Nava
 * @date 2023
 */
#include <UKFModel.h>
#include <UnscentedKF.h>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("UKF TEST")
{
    // system initialization
    int nState = 2;
    int nInput = 1;
    int nMeasures = 1;

    Eigen::VectorXd u(nInput);
    Eigen::VectorXd x0(nState);
    Eigen::VectorXd y(nMeasures);
    Eigen::MatrixXd Q(nState, nState);
    Eigen::MatrixXd R(nMeasures, nMeasures);
    Eigen::MatrixXd P(nState, nState);

    // set initial conditions
    x0.setZero(); // initial state
    Q.setIdentity(); // process noise
    R.setIdentity(); // measurements noise
    P.setIdentity(); // initial covariance matrix
    u.setZero(); // exogenous input
    y.setZero(); // measurements

    // initialize a shared pointer to an instance of the JetSystemModel class
    std::shared_ptr<UKFModel> model = std::make_shared<UKFModel>(nState, nMeasures);

    // create an instance of the UKF class
    UnscentedKF ukf(model, P, Q, R);
    ukf.init(x0);

    // get the first estimated state
    Eigen::VectorXd x_est(nState);

    ukf.predict();
    ukf.update(y);
    x_est = ukf.getState();

    // use ctest --output-on-failure to visualize this message in case of failure
    INFO("UKF output is: " << x_est);

    // expected solution
    Eigen::VectorXd check_solution;
    check_solution.resize(nState);
    check_solution.setZero();

    // check that computed values match expected values
    REQUIRE(x_est.isApprox(check_solution, 0.001));
}
