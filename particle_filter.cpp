//
// Created by jaysh on 4/29/2018.
//

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <float.h>
#include <time.h>
#include "particle_filter.h"
#include "Eigen/Dense"
//#include "Eigen"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

ParticleFilter::ParticleFilter() {}

ParticleFilter::~ParticleFilter() {}

double gaussian(double mu, double sigma, double x)
{
  double num = (exp(-pow((x-mu), 2))/(2.0*pow(sigma, 2)));
  double den = sqrt(2.0*3.14*(pow(sigma, 2)));
  double gauss = num/den;
  return gauss;
}

void ParticleFilter::perform_pf(double x, double y) {
  VectorXd current_meas = VectorXd(2);
  current_meas << x, y;
  if (!is_initialized)
  {
    num_particles = 25;  /* number of particles which is a hyper-parameter */
    default_random_engine gen; /* random_generator  */
    previous_meas = VectorXd(2);
    double std[] = {0.8, 0.8};
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);

    // =========================================================
    //  Creating particles and their state normally distributed
    // ======================================================
    for (unsigned int i = 0; i < num_particles; i++)
    {
      Particle particle;
      particle.id = i;
      particle.x = dist_x(gen);
      particle.y = dist_y(gen);
      particle.weight = 1.0;
      particles.push_back(particle);
    }
    previous_meas << x, y;
    is_initialized = true;
    disp_particle(particles);
    return;
  }

  for (unsigned int i = 0; i < num_particles; i++)
  {
    particles.at(i).x = particles.at(i).x + current_meas(0) - previous_meas(0);
    particles.at(i).y = particles.at(i).y + current_meas(1) - previous_meas(1);

  }
  VectorXd sense_meas = VectorXd(land_mrks.size());

  for (unsigned int i = 0; i < land_mrks.size(); i++)
  {
    float x = current_meas(0);
    float y = current_meas(1);
    vector<float> land_mrk = land_mrks.at(i);
    float dist = sqrt(pow(x-land_mrk.at(0), 2)+pow(y-land_mrk.at(1), 2));
    sense_meas(i) = dist;
  }
  updateWeights(sense_meas, current_meas);
  re_sample();
  previous_meas = current_meas;
  disp_particle(particles);
  return;
}

void ParticleFilter::updateWeights(const Eigen::VectorXd &sense_meas, const Eigen::VectorXd &current_meas)
{
  float tot = 0;
  for (unsigned int i = 0; i < num_particles; i++)
  {
    particles.at(i).weight = measure_prob(particles.at(i), sense_meas, current_meas);
    tot = tot + particles.at(i).weight;
  }

  for (unsigned int i = 0; i < num_particles; i++)
  {
    particles.at(i).weight = particles.at(i).weight/tot;
  }
}

double ParticleFilter::measure_prob(const Particle &current_particle, const Eigen::VectorXd &sense_meas, const Eigen::VectorXd &measurement)
{
  srand(time(NULL));
  double prob = 1.0;
  double dist;
  double random_number;
  for (unsigned int i = 0; i < land_mrks.size(); i++)
  {
    float x = measurement(0);
    float y = measurement(1);
    vector<float> land_mrk = land_mrks.at(i);
    dist = sqrt(pow(current_particle.x - land_mrk.at(0), 2)+pow(current_particle.x - land_mrk.at(1), 2));
    double den = sense_meas(i);
    random_number =  rand()%(int)(dist+den);
    //std::cout << "random_number = " << i << " " << random_number << std::endl;
    prob = prob * random_number/sense_meas(i);
  }
  // std::cout << "============================ " << std::endl;
  return prob;
}

void ParticleFilter::re_sample()
{
  std::vector<Particle> new_particles;
  srand(time(NULL));
  for (unsigned int i = 0; i < num_particles; i++)
  {
    double sum_alpha = 0.0;
    double random_number;

    random_number = (double)rand() / (double)(RAND_MAX);
    for (unsigned int j = 0; j < num_particles; j++)
    {
      sum_alpha = sum_alpha + particles.at(j).weight;
      // std::cout << "sum_alpha = " << sum_alpha << ", random_number = " << random_number << std::endl;
      if (sum_alpha >= random_number)
      {
        new_particles.push_back(particles.at(j));
        break;
      }
    }
  }
  particles = new_particles;
}

void ParticleFilter::disp_particle(const vector<Particle> & vect)
{
  double highest_weight = INT_MIN;
  vector<Particle>::const_iterator it;
  for (it = vect.begin() ; it != vect.end(); ++it)
  {
    Particle particle = *it;
    if (particle.weight > highest_weight)
    {
      highest_weight = particle.weight;
      best_particle = particle;
    }
    // std::cout << "id = " << particle.id << ", x = " << particle.x << ", y = " << particle.y << ", weight = " << particle.weight << std::endl;
  }
  //std::cout << "best_particle" << " x = " << best_particle.x << ", y = " << best_particle.y << std::endl;

}




