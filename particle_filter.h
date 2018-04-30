//
// Created by jaysh on 4/29/2018.
//

#ifndef TELENAVSENSORFUSION_PARTICLE_FILTER_H
#define TELENAVSENSORFUSION_PARTICLE_FILTER_H

#include <iostream>
#include <vector>
//#include "Eigen"
#include "Eigen/Dense"

using namespace std;

struct Particle {

  int id;
  double x;
  double y;
  double weight;
};

class ParticleFilter
{
public:

  Particle best_particle;

  Eigen::VectorXd previous_meas;

  vector<vector<float>> land_mrks = {{-50, -25}, {50, 25}};

  int num_particles;

  bool is_initialized;

  std::vector<Particle> particles;

  /**
   * Constructor
   */
  ParticleFilter();

  /**
   * Destructor
   */
  virtual ~ParticleFilter();

  /**
	 * Performing Particle Filter
	 * @param x: x-position of the vehicle
	 * @param y: y-position of the vehicle
	 */
  void perform_pf(double x, double y);

  /**
	 * Calculating the importance of particle. This function is a helper function to updateWeights
	 * @param current_particle: current particle whose weight has to be updated
	 * @param sense_meas: distance measurement from landmarks of actual vehicle
   * @param measurement: distance measurement from landmarks of current particle
	 */
  double measure_prob(const Particle &current_particle, const Eigen::VectorXd &sense_meas, const Eigen::VectorXd &measurement);

  /**
	 * Sampling important particles to predict the vehicle
	 */
  void re_sample();

  /**
	 * Update weights of the particles
	 * @param sense_meas: distance measurement from landmarks of actual vehicle
   * @param current_meas: current measurement given by the Lidar or Radar sensor
	 */
  void updateWeights(const Eigen::VectorXd &sense_meas, const Eigen::VectorXd &current_meas);


  void disp_particle(const vector<Particle> & vect);

};


#endif //TELENAVSENSORFUSION_PARTICLE_FILTER_H
