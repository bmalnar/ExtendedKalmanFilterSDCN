#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  static const bool print_flag_ = false;
  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to normalize an angle to be within range (-pi,pi)
  */
  float NormalizeAngle(float angle);

  /**
  * A helper method to calculate the h function for radar
  */
  VectorXd CalculateRadarHFunction(const VectorXd& x_state);

  /**
  * A helper method to print when debug flag is true
  */
  static void PrintDebug(const std::string s);
  static void PrintDebugMatrix(const std::string s, const MatrixXd mat);
  static void PrintDebugVector(const std::string s, const VectorXd vec);
};

#endif /* TOOLS_H_ */
