#ifndef JET_SYSTEM_MODEL_H
#define JET_SYSTEM_MODEL_H

#include "SystemModel.h"

class JetSystemModel : public SystemModel
{
public:
  // overwrite constructor for the child class
  JetSystemModel(int _nMeasures, double _dt);

  // initialize variables of the child class
  double dt = 0.01;
  int nMeasures = 1;

  // state model parameters
  static Eigen::Matrix2d A;
  static Eigen::Vector2d B;

  // measurement model parameters
  static Eigen::Vector2d h;

  // add additional methods and override existing methods
  void configureModelParameters();
  Eigen::VectorXd dynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u) override;
  Eigen::VectorXd observationModel(const Eigen::VectorXd &x) override;
};

#endif /* end of include guard JET_SYSTEM_MODEL_H */
