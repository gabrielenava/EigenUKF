/**
 * @file main.cpp
 * @author Gabriele Nava
 * @date 2023
 */
#include <memory>
#include <iostream>
#include <fstream>
#include <UnscentedKF.h>
#include <SystemModel.h>

int main(int argc, char const *argv[])
{
	// open the file containing the example data
	std::ifstream inputFile("example_data.txt");

	if (!inputFile.is_open())
	{
		std::cerr << "Failed to open the file." << std::endl;
		return 1;
	}

	// store the thrusts and time data
	std::vector<double> thrustData;
	std::vector<double> timeData;
	std::string line;

	// read and ignore the header line
	std::getline(inputFile, line);

	while (std::getline(inputFile, line))
	{
		std::istringstream iss(line); // treat line as an input
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

	// step time
	double dt = 0.01;

	// system initialization
	Eigen::VectorXd u(1);
	Eigen::VectorXd x0(2);
	Eigen::MatrixXd Q(2, 2);
	Eigen::MatrixXd R(1, 1);
	Eigen::MatrixXd P(2, 2);

	// set initial conditions
	x0.setZero();			   // initial state
	Q << 0.01, 0.0, 0.0, 0.01; // process noise
	R << 100.0;				   // measurements noise
	P << 1.0, 0.0, 0.0, 1.0;   // initial covariance matrix
	u << 0.0;				   // exogenous input

	// initialize a pointer to an instance of the SystemModel class
	std::shared_ptr<SystemModel> model = std::make_shared<SystemModel>(R, Q, dt);

	// create an instance of the UKF class
	UnscentedKF ukf(model, P);
	ukf.init(x0);

	// iterate on the data and get the estimated state
	Eigen::VectorXd y(1);
	std::vector<Eigen::VectorXd> res;

	for (int k = 0; k < timeData.size(); k++)
	{
		// update the measurement
		y[0] = thrustData[k];

		// call to the UKF
		ukf.predict(u, dt);
		ukf.update(y);
		res.push_back(ukf.get_state());
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
	for (const Eigen::VectorXd &row : res)
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
