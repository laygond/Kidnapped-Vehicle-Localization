/**
 * particle_filter.cpp
 *
 */

#include "particle_filter.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
//Note: the vector that holds all "particles" is public to be used among all ParticleFilter methods

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /****************
   * Initialization
   ****************/
  
  num_particles = 100;  // Set number of particles (private field from ParticleFilter class)
    
  // This line creates a normal (Gaussian) distribution for x,y,theta with their corresponding std
  normal_distribution<double> distrib_x(x, std[0]);
  normal_distribution<double> distrib_y(y, std[1]);
  normal_distribution<double> distrib_theta(theta, std[2]);
  
  //Create random particle near GPS coordinates and add them to vector of particles
  std::default_random_engine gen;
  for (int i = 0; i < num_particles; ++i){
    Particle aParticle;
    aParticle.id = i;
    aParticle.x = distrib_x(gen);  //Add random Gaussian noise to each particle
    aParticle.y = distrib_y(gen);
    aParticle.theta = distrib_theta(gen);
    aParticle.weight = 1.0;        // Set weight to 1
    
    particles.push_back(aParticle);
    weights.push_back(aParticle.weight);  //private variable from the Particle filter Class
  }
  
  //Set initialized variable as complete
  is_initialized = true;
}

    
//     // Print your samples to the terminal.
//     std::cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " 
//               << sample_theta << std::endl;

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /****************
   * Prediction 
   ****************/
  
  // This line creates a normal (Gaussian) distribution for movement x,y,theta with their corresponding std
  normal_distribution<double> distrib_move_x(0, std_pos[0]);
  normal_distribution<double> distrib_move_y(0, std_pos[1]);
  normal_distribution<double> distrib_move_theta(0, std_pos[2]);

  // Calculate the Prediction Movement for each particle
  std::default_random_engine gen;
  if (yaw_rate < 0.00001)
  {
    for (int i = 0; i < num_particles; ++i){
      particles[i].x = particles[i].x + velocity/yaw_rate *(-sin(particles[i].theta) + sin(particles[i].theta + yaw_rate*delta_t)) +  distrib_move_x(gen);
      particles[i].y = particles[i].y + velocity/yaw_rate *( cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t)) +  distrib_move_y(gen);
      particles[i].theta = particles[i].theta + yaw_rate*delta_t +  distrib_move_theta(gen);
    }
  }
  else{}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map) {
  /****************
   * Update 
   ****************/
  
  if (observations.size() > 0) // Only update if there are observations
  { 
    // For each particle place robot's observations on the particle's reference frame
    // then transform each particle's observation into map reference frame (this will be the x and y of gaussian)
    // and associate closest landmark (this will be the u_x and u_y of gaussian)
    // and calculate particle's observation weight using gaussian formula.
    // finally calculate particle's final weight (product of all observation weights)
    for (size_t i=0; i<particles.size(); ++i){
      double measurement_prob = 1.0; // declare raw weight value for final weight of particle

      for (size_t k=0; k<observations.size(); ++k){
        // Transform observation to particles perspective in map coordinates
        double x_obs_transf, y_obs_transf; 
        x_obs_transf = particles[i].x + (cos(particles[i].theta) * observations[k].x) - (sin(particles[i].theta) * observations[k].y);
        y_obs_transf = particles[i].y + (sin(particles[i].theta) * observations[k].x) + (cos(particles[i].theta) * observations[k].y);
        particles[i].sense_x.push_back(x_obs_transf);
        particles[i].sense_y.push_back(y_obs_transf);

        // Find closest distance between transformed observation and landmarks within sensor range
        double dist_min = std::numeric_limits<const double>::infinity(); // minimum distance of landmark to otransformed obs
        int  closest_lm_idx; //closest landmark index
        for (size_t m=0; m<map.landmark_list.size(); ++m){
          // if landmark within particle's sensor range
          if(dist( particles[i].x,  particles[i].y, map.landmark_list[m].x_f, map.landmark_list[m].y_f) < sensor_range)
          {
             // Save particle's closest landmark from transformed observation
             double dist_lm_Tobs;      //distance between a landmark and transformed observation
             dist_lm_Tobs = dist(x_obs_transf, y_obs_transf, map.landmark_list[m].x_f, map.landmark_list[m].y_f);
             if (dist_lm_Tobs<dist_min)
             {
                closest_lm_idx = m;  
                dist_min = dist_lm_Tobs;
             }
          }       
        }//end of landmark loop
        if (dist_min == std::numeric_limits<const double>::infinity())
        {// then no landmark is within particle sensor range and no need to keep checking observations
          particles[i].weight = 0;
          break; // observation loop
        }//else store index of closest landmark to transformed observation
        particles[i].associations.push_back(closest_lm_idx);

        // Calculate observation weight and accumulate for final weight
        measurement_prob *= multiv_prob(std_landmark[0], std_landmark[1], x_obs_transf, y_obs_transf, map.landmark_list[closest_lm_idx].x_f, map.landmark_list[closest_lm_idx].y_f);

      }//end of observation loop
      
      // Set particle weight
        particles[i].weight = measurement_prob;
        weights[i] = measurement_prob;
      
    }//end of particle loop
    //RESET 
    dist_min
  }// end of firstif statement
}


void ParticleFilter::resample() {
  /****************
   * Resample 
   ****************/
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  // This line creates a weighted distribution according to the weights
  std::discrete_distribution<int> distrib_index(weights.begin(), weights.end());
  
  //Choose a random particle from distribution to add to resampled particle vector
  std::vector<Particle> resampled_particles;
  std::default_random_engine gen;
  for (int i = 0; i < num_particles; ++i){
    
    resampled_particles.push_back(particles[distrib_index(gen)]);
  }
  particles = resampled_particles;
}
