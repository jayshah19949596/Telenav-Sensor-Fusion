//
// Created by jaysh on 4/28/2018.
//

#ifndef TELENAVSENSORFUSION_ITERATION_H
#define TELENAVSENSORFUSION_ITERATION_H

#include "Eigen/Dense"

// #include "Eigen"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Iteration
{
public:
  VectorXd L_meas;
  VectorXd R_meas;
  float vel_L, vel_R, time_L, time_R;
  /**
  * Constructor.
  */
  Iteration();

  /**
  * Destructor.
  */

  virtual ~Iteration();

  /**
 * Returns the Euclidean Distance between two vector
 * @param L: Lidar Measurement Vector
 * @param R: Radar Measurement Vector
 */
  float get_distance(const VectorXd &L, const VectorXd &R);

  /**
  * Selects the best pair of Lidar measurement and Radar measurement based on
  * the minimum Euclidean distance between the Lidar and Radar measurement pair
  * @param L: Lidar Measurement Vector
  * @param R: Radar Measurement Vector
  */
  void select_best_pair(const MatrixXd &L_mat, const MatrixXd &R_mat);

};


#endif //TELENAVSENSORFUSION_ITERATION_H
