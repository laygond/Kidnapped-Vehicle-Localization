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
  for (int i = 0; i < num_particles; ++i){
    particles[i].x = particles[i].x + velocity/yaw_rate *(-sin(particles[i].theta) + sin(particles[i].theta + yaw_rate*delta_t)) +  distrib_move_x(gen);
    particles[i].y = particles[i].y + velocity/yaw_rate *( cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t)) +  distrib_move_y(gen);
    particles[i].theta = particles[i].theta + yaw_rate*delta_t +  distrib_move_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  if (observations.size() > 0) // Only update if there are observations
  { 
    for (size_t i=0; i<particles.size(); ++i){
      double measurement_prob = 1.0; // declare raw weight value for a particle

      for (size_t k=0; k<observations.size(); ++k){

        //Define case where no landmark can be associated to transformed observation
        particles[i].associations.push_back(-1);  // negative id
        double dist_min = std::numeric_limits<const double>::infinity();
        particles[i].sense_x.push_back(dist_min);
        particles[i].sense_y.push_back(dist_min);

        // Find closest distance between transformed observation and landmarks within sensor range
        for (size_t m=0; m<map.landmark_list.size(); ++m){
          if(dist( particles[i].x,  particles[i].y, map.landmark_list[m].x_f, map.landmark_list[m].y_f) < sensor_range)
          {
             // transform observation to particles perspective in map coordinates
             double x_obs_transf, y_obs_transf; 
             x_obs_transf = particles[i].x + (cos(particles[i].theta) * observations[k].x) - (sin(particles[i].theta) * observations[k].y);
             y_obs_transf = particles[i].y + (sin(particles[i].theta) * observations[k].x) + (cos(particles[i].theta) * observations[k].y);

             // Set particle to closest landmark from transformed observation
             double dist_ml_Tobs;      //distance between a landmark and transformed observation
             dist_ml_Tobs = dist(x_obs_transf, y_obs_transf, map.landmark_list[m].x_f, map.landmark_list[m].y_f);
             if (dist_ml_Tobs<dist_min)
             {
                particles[i].associations[k] = map.landmark_list[m].id_i;  
                particles[i].sense_x[k] = map.landmark_list[m].x_f;
                particles[i].sense_y[k] = map.landmark_list[m].y_f;
                dist_min = dist_ml_Tobs;
             }
          }       
        }//end of landmark loop

        // Calculate weight
        measurement_prob *= multiv_prob(std_landmark[0], std_landmark[1], observations[k].x, observations[k].y, particles[i].sense_x[k], particles[i].sense_y[k]);

      }//end of observation loop
      
      // Set particle weight
        particles[i].weight = measurement_prob;
        weights[i] = measurement_prob;
      
    }//end of particle loop
  }// end of firstif statement
}

void ParticleFilter::resample() {
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

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}