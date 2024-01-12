/**
 * @file SystemModel.h
 * @author Gabriele Nava
 * @date 2023
 */
#ifndef SYSTEM_MODEL_H
#define SYSTEM_MODEL_H

#include <Eigen/Dense>

class SystemModel
{
  /*
    This class is a parent class for the UKF.
    The class contains two default methods: dynamicsModel and observationModel. The user can create a child class
    and modify the content of dynamicsModel and observationModel methods, as well as add new methods and variables
    as needed. He cannot modify the interfaces of dynamicsModel and observationModel methods, because they are needed
    for the UnscentedKF class to work properly.
  */
public:
  // default constructors
  SystemModel();
  SystemModel(int _nMeasures);

  // default destructor
  ~SystemModel();

protected:
  // initialize class variables
  int nMeasures = 1;

  // class default methods (inputs and outputs are protected to ensure compatibility with UnscentedKF class)
  virtual Eigen::VectorXd dynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u);
  virtual Eigen::VectorXd observationModel(const Eigen::VectorXd &x);

  friend class UnscentedKF;
};

#endif /* end of include guard SYSTEM_MODEL_H */
