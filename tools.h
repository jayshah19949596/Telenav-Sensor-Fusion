#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
// #include "Eigen"
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

  vector<string> split(const string &s, char delim);

  /**
  * Generic function to display vector holding any data type
  */
  template <typename T>
  void disp_vect(const vector<T> & vect)
  {
    typename vector<T>::const_iterator it;
    for (it = vect.begin() ; it != vect.end(); ++it)
    {
      std::cout << ' ' << *it;
    }
    std::cout << '\n';
  }


  void prepare_data(float &x, float &y, float &velocity, float &sensor, float &time, const vector<string> &data_vector);

  /**
  * Prepare Lidar and Radar Matrix for one iteration
  * @param state_mat: Lidar Measurement Matrix or Radar Measurement matrix
  * @param data_vector: line element of data txt file
  */
  void prepare_mat(MatrixXd &state_mat,const vector<string> &data_vector);


};

#endif /* TOOLS_H_ */
