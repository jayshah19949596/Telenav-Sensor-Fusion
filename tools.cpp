#include <iostream>
#include "tools.h"
// #include <Eigen>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

void Tools::prepare_data(float &x, float &y, float &velocity, float &sensor, float &time, const vector<string> &data_vector)
{
  string check = ":";
  int pos;
  string data;
  vector<string>::iterator it;
  stringstream iss;

  // x
  data = data_vector.at(0);
  pos = data.find(check);
  // iss << data.substr(pos+1, data.size());
  // iss >> x;
  x = stof(data.substr(pos+1, data.size()));

  //std::cout << "x =  "<< x << " pos = " << pos << std::endl;

  // y
  data = data_vector.at(1);
  pos = data.find(check);
  y = stof(data.substr(pos+1, data.size()));

  // velocity
  data = data_vector.at(2);
  pos = data.find(check);
  velocity = stof(data.substr(pos+1, data.size()));

  // sens
  data = data_vector.at(3);
  pos = data.find(check);
  sensor = stof(data.substr(pos+1, data.size()));

  // model
  data = data_vector.at(4);
  pos = data.find(check);

  // time
  data = data_vector.at(5);
  pos = data.find(check);
  time = stof(data.substr(pos+1, data.size()));
}


void Tools::prepare_mat(MatrixXd &state_mat, const vector<string> &data_vector)
{
  string check = ":";
  string data;
  int pos;

  data = data_vector.at(0);
  pos = data.find(check);
  state_mat(state_mat.rows()-1, 0) = stof(data.substr(pos+1, data.size()));

  data = data_vector.at(1);
  pos = data.find(check);
  state_mat(state_mat.rows()-1, 1) = stof(data.substr(pos+1, data.size()));

  data = data_vector.at(2);
  pos = data.find(check);
  state_mat(state_mat.rows()-1, 2) = stof(data.substr(pos+1, data.size()));

  data = data_vector.at(5);
  pos = data.find(check);
  state_mat(state_mat.rows()-1, 3) = stof(data.substr(pos+1, data.size()));

}


vector<string> Tools::split(const string &s, char delim)
{
  stringstream ss(s);
  string item;
  vector<string> tokens;
  while (getline(ss, item, delim))
  {
    tokens.push_back(item);
  }
  return tokens;
}