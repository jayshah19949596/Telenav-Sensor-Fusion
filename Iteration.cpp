#include "Iteration.h"
#include <iostream>
#include "tools.h"
#include <limits>
#include <math.h>
// #include <Eigen>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;


Iteration::Iteration() {}

Iteration::~Iteration() {}

float Iteration::get_distance(const VectorXd &L, const VectorXd &R)
{
  VectorXd dist_mat;
  float distance = 0;
  // std::cout << "L = " << L.transpose() << std::endl;
  // std::cout << "R = " << R.transpose() << std::endl;
  dist_mat = L - R;
  // std::cout << "dist_mat before square L - R = " << dist_mat.transpose() << std::endl;
  dist_mat = dist_mat.array().square();
  // std::cout << "dist_mat after square = " << dist_mat.transpose() << std::endl;
  distance = dist_mat.sum();
  // std::cout << "sum distance = " << distance << std::endl;
  distance = sqrt(distance);
  // std:cout << "===============" << std::endl;
  return distance;
}

void Iteration::select_best_pair(const MatrixXd &L_mat, const MatrixXd &R_mat)
{
  float best_distance = std::numeric_limits<float>::max();
  float current_distance;
  VectorXd L, R;

  L = VectorXd(2);
  R = VectorXd(2);
//  std::cout << "number of Lidar measurements = " << L_mat.rows() << std::endl;
//  std::cout << "number of Radar measurements = " << R_mat.rows() << std::endl;
  for(int i = 0; i < L_mat.rows(); i++)
  {
    // std::cout << "i = " << i << std::endl;
    L << L_mat(i, 0), L_mat(i, 1);
    for(int j = 0; j < R_mat.rows(); j++)
    {
      // std::cout << "j = " << j << std::endl;

      R << R_mat(j, 0), R_mat(j, 1);
      current_distance = get_distance(L, R);
//      if (fabs(L(1) - R(1)) == 1.0)
//      {
//        std::cout << "L(1) - R(1) = 1, L(1) = " << L(1) << " R(1) = " << R(1) << std::endl;
//      }
      if (best_distance > current_distance)
      {
        best_distance = current_distance;
        L_meas = L ;
        R_meas = R;
        time_L = L_mat(j, 3);
        time_R = R_mat(j, 3);
        vel_L = L_mat(j, 2);
        vel_R = R_mat(j, 2);
      }
    }
  }
//  std::cout << "L_meas = " << L_meas.transpose() << std::endl;
//  std::cout << "R_meas = " << R_meas.transpose() << std::endl;
//  std::cout << "best_distance = " << best_distance << std::endl;
//  std:cout << "===============" << std::endl;

}


