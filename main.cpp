#include <iostream>
// #include <Eigen>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "tools.h"
#include "Iteration.h"
#include "kalman_filter.h"
#include "particle_filter.h"
#include "Eigen/Dense"

using namespace std;


int main(int argc, char** argv)
{

  // Have to give absolute path to the txt file
  const std::string DATA_FILE_PATH = "C:\\Users\\jaysh\\CLionProjects\\TelenavSensorFusion\\SensorFusionProblem\\SensorFusion-data-1.txt";
  // const std::string DATA_FILE_PATH = "SensorFusion-data-1.txt";
  string line, itr_check = "iteration";
  vector <string> split_vector;
  MatrixXd L_mat, R_mat;
  int itr_no = 0;
  float sensor;
  Tools tools;
  KalmanFilter kf;
  ParticleFilter pf;
  MeasurementPackage meas_package;
  Iteration itr;
  // string filter_method = "kalman_filter";
  // string filter_method = "particle_filter";
  string filter_method = argv[1];
  std::cout << "filter_method = " << filter_method << std::endl;

  // ===================================
  // Read the measurement line by line
  // ===================================
  std::ifstream in_file(DATA_FILE_PATH);

  while(getline(in_file, line))
  {

    int check = line.find(itr_check);
    if (check == 0)
    {
      if (itr_no != 0)
      {

        itr.select_best_pair(L_mat, R_mat);

        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = itr.L_meas;
        meas_package.timestamp_ = itr.time_L;

        if (filter_method == "kalman_filter")
        {
          kf.perform_kf(meas_package);
          std::cout << "State vector = " << kf.x_.transpose() << std::endl;
        }
        else if (filter_method == "particle_filter")
        {
          pf.perform_pf(meas_package.raw_measurements_(0), meas_package.raw_measurements_(1));
          std::cout << "State vector = " << " x = " << pf.best_particle.x << ", y = " << pf.best_particle.y << std::endl;
        }
        std::cout << "Lidar measurement = " << itr.L_meas.transpose() << std::endl;
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = itr.R_meas;
        meas_package.timestamp_ = itr.time_R;

        if (filter_method == "kalman_filter")
        {
          kf.perform_kf(meas_package);
          std::cout << "State vector = " << kf.x_.transpose() << std::endl;
        }
        else if (filter_method == "particle_filter")
        {
          pf.perform_pf(meas_package.raw_measurements_(0), meas_package.raw_measurements_(1));
          std::cout << "State vector = " << " x = " << pf.best_particle.x << ", y = " << pf.best_particle.y << std::endl;
        }
        std::cout << "Radar measurement = " << itr.R_meas.transpose() << std::endl;
        std::cout << " ===========" << "iteration number = " << itr_no <<"============ "<< std::endl;

      }
      L_mat = MatrixXd(0, 4);
      R_mat = MatrixXd(0, 4);
      itr_no  = itr_no + 1;
    }
    else
    {
      istringstream iss(line);
      split_vector = tools.split(line, ',');
      string data = split_vector.at(3);
      int pos = data.find(":");
      sensor = stof(data.substr(pos+1, data.size()));

      if (sensor == 2)
      {
        // ========
        //  Radar
        // ========
        R_mat.conservativeResize(R_mat.rows()+1, R_mat.cols());
        tools.prepare_mat(R_mat, split_vector);
      }
      else if (sensor == 4)
      {
        // ========
        //  Lidar
        // ========
        L_mat.conservativeResize(L_mat.rows()+1, L_mat.cols());
        tools.prepare_mat(L_mat, split_vector);

      }
    }
  }

  return 0;
}


