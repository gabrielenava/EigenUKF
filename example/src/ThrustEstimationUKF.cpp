/**
 * @file ThrustEstimationUKF.cpp
 * @author Gabriele Nava
 * @date 2023
 */
#include <JetUKFModel.h>
#include <UnscentedKF.h>
#include <fstream>
#include <iostream>
#include <memory>

/*
  This example illustrates how to use UKFModel and UnscentedKF, the two classes composing Eigen_UKF
  library. In this example, the UKF is used to estimate the thrust provided by a jet engine and its
  rate of change.
*/
int main(int argc, char const* argv[])
{
    std::cout << "Loading data..." << std::endl;

    // open the file containing the example data
    std::ifstream inputFile("jetData.txt");

    if (!inputFile.is_open())
    {
        std::cerr << "Failed to open the jet data file." << std::endl;
        return 1;
    }

    // create variables to store the thrusts and time data
    std::vector<double> thrustData;
    std::vector<double> timeData;
    std::string line;

    // read and ignore the header line
    std::getline(inputFile, line);

    while (std::getline(inputFile, line))
    {
        // treat line as an input string
        std::istringstream iss(line);
        std::string thrustStr, timeStr;

        if (std::getline(iss, thrustStr, ',') && std::getline(iss, timeStr, ','))
        {
            double thrustValue = std::stod(thrustStr);
            double timeValue = std::stod(timeStr);

            thrustData.push_back(thrustValue);
            timeData.push_back(timeValue);
        }
    }

    // close the file
    inputFile.close();

    std::cout << "Data loaded!" << std::endl;
    std::cout << "Running Estimation..." << std::endl;

    // step time
    double dt = 0.01;

    // system initialization
    int nState = 2;
    int nInput = 1;
    int nMeasures = 1;

    Eigen::VectorXd u(nInput);
    Eigen::VectorXd x0(nState);
    Eigen::MatrixXd Q(nState, nState);
    Eigen::MatrixXd R(nMeasures, nMeasures);
    Eigen::MatrixXd P(nState, nState);

    // set initial conditions
    x0.setZero(); // initial state
    Q << 0.01, 0.0, 0.0, 0.01; // process noise
    R << 100.0; // measurements noise
    P << 1.0, 0.0, 0.0, 1.0; // initial covariance matrix
    u << 0.0; // exogenous input

    // initialize a shared pointer to an instance of the JetUKFModel class
    std::shared_ptr<JetUKFModel> model = std::make_shared<JetUKFModel>(dt);
    model->updateProcessModel(u);

    // create an instance of the UKF class
    UnscentedKF ukf(model, P, Q, R);
    ukf.init(x0);

    // iterate on the data and get the estimated state
    Eigen::VectorXd y(1);
    std::vector<Eigen::VectorXd> x_est;

    for (int k = 0; k < timeData.size(); k++)
    {
        // update the measurement
        y[0] = thrustData[k];

        // call to the UKF
        ukf.predict();
        ukf.update(y);
        x_est.push_back(ukf.getState());
    }

    std::cout << "Estimation done!" << std::endl;

    // saving back the estimated state to a .txt file
    std::ofstream outputFile("outputUKF.txt");

    if (!outputFile.is_open())
    {
        std::cerr << "Failed to open the output file." << std::endl;
        return 1;
    }

    // iterate through the vector and write each Eigen vector as a row
    for (const Eigen::VectorXd& row : x_est)
    {
        for (int i = 0; i < row.size(); ++i)
        {
            // write the i-th element
            outputFile << row(i);

            if (i < row.size() - 1)
            {
                // add a delimiter between elements
                outputFile << ",";
            }
        }
        // start a new line for the next row
        outputFile << "\n";
    }

    // close the output file
    outputFile.close();

    std::cout << "Results saved in outputUKF.txt" << std::endl;

    return 0;
}
