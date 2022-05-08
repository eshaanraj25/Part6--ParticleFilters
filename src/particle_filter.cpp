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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Set number of particles
  num_particles = 100;
  
  // Get standard deviations and initialize random engine
  std :: default_random_engine gen;
  double std_x, std_y, std_theta;
  
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];
  
  // Generate normal distributions
  std :: normal_distribution<double> dist_x(x, std_x);
  std :: normal_distribution<double> dist_y(y, std_y);
  std :: normal_distribution<double> dist_theta(theta, std_theta);
  
  // Generate the particles from distribution
  for (int i = 0; i < num_particles; i += 1){
    // Get sampled x, y and theta
    double sample_x, sample_y, sample_theta;
    
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);
    
    // Generate particle
    Particle particle;
    particle.id = i;
    particle.x = sample_x;
    particle.y = sample_y;
    particle.theta = sample_theta;
    particle.weight = 1;
    
    particles.push_back(particle);
    weights.push_back(1);
  }

  std :: cout << "Initialized " << num_particles << " particles" << std :: endl;
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  // Get standard deviations and initialize random engine
  std :: default_random_engine gen;
  double std_x, std_y, std_theta;
  
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];
  
  // Generate normal distributions
  std :: normal_distribution<double> dist_x(0, std_x);
  std :: normal_distribution<double> dist_y(0, std_y);
  std :: normal_distribution<double> dist_theta(0, std_theta);
  
  // Calculate xf, yf and thetaf for each particle
  // and assign them respectively
  double x, y, theta, xf, yf, thetaf;
  for (int i = 0; i < num_particles; i += 1){
    x = particles[i].x;
    y = particles[i].y;
    theta = particles[i].theta;
    
    if (fabs(yaw_rate) < 1e-5){
      xf = x + velocity * delta_t * std :: cos(theta);
      yf = y + velocity * delta_t * std :: sin(theta);
      thetaf = theta;
    }
    else {
      xf = x + (velocity / yaw_rate) * (std :: sin(theta + yaw_rate * delta_t) - std :: sin(theta));
      yf = y + (velocity / yaw_rate) * (std :: cos(theta) - std :: cos(theta + yaw_rate * delta_t));
      thetaf = theta + yaw_rate * delta_t;
    }
    
    particles[i].x = xf + dist_x(gen);
    particles[i].y = yf + dist_y(gen);
    particles[i].theta = thetaf + dist_theta(gen);
  }
  
  std :: cout << "Accomplished Prediction Step" << std :: endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  int observations_size, predicted_size;
  observations_size = observations.size();
  predicted_size = predicted.size();
  
  // Loop over each observation
  double best_difference, difference;
  int best_difference_id;
  for (int i = 0; i < observations_size; i += 1) {
    LandmarkObs observation = observations[i];
    best_difference = 1e10;
    difference = 0;
    best_difference_id = 0;
    
    // Loop over each prediction
    for (int j = 0; j < predicted_size; j += 1) {
      LandmarkObs prediction = predicted[j];
      difference = dist(observation.x, observation.y, prediction.x, prediction.y);
      if (difference < best_difference) {
        best_difference = difference;
        best_difference_id = prediction.id;
      }
    }
    
    // Assign each observation an associated id
    observations[i].id = best_difference_id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  // Read parameters
  double std_x, std_y;
  
  std_x = std_landmark[0];
  std_y = std_landmark[1];
  
  // Update weights for each particle
  double x, y, theta;
  int landmark_list_size, observations_size;
  
  landmark_list_size = map_landmarks.landmark_list.size();
  observations_size = observations.size();
  
  for(int p = 0; p < num_particles; p += 1){
    x = particles[p].x;
    y = particles[p].y;
    theta = particles[p].theta;
    
    // Get actual Landmark locations near the particle
    vector<LandmarkObs> actualLandmarks;
    double x_map, y_map, difference;
    int id;
    for (int i = 0; i < landmark_list_size; i += 1){
      x_map = map_landmarks.landmark_list[i].x_f;
      y_map = map_landmarks.landmark_list[i].y_f;
      id = map_landmarks.landmark_list[i].id_i;

      difference = dist(x, y, x_map, y_map);
      if (difference <= sensor_range){
        actualLandmarks.push_back(LandmarkObs {id, x_map, y_map});
      }
    }

    // Convert the given observations to Global coordinates
    vector<LandmarkObs> observedLandmarks;
    double x_obs, y_obs;
    for (int i = 0; i < observations_size; i += 1){
      x_obs = x + cos(theta) * observations[i].x - sin(theta) * observations[i].y;
      y_obs = y + sin(theta) * observations[i].x + cos(theta) * observations[i].y;
      
      observedLandmarks.push_back(LandmarkObs{observations[i].id, x_obs, y_obs});
    }
    
    // Associate the observations and actual landmarks
    dataAssociation(actualLandmarks, observedLandmarks);
    
    // Go over each observation
    particles[p].weight = 1.0;
    double observedX, observedY, landmarkX, landmarkY;
    int observedId;
    
    for (int i = 0; i < observations_size; i += 1){
      observedX = observedLandmarks[i].x;
      observedY = observedLandmarks[i].y;
      observedId = observedLandmarks[i].id;
      
      // Get the associated landmark
      int actualLandmarks_size = actualLandmarks.size();
      for (int j = 0; j < actualLandmarks_size; j += 1){
        if (actualLandmarks[j].id == observedId){
          landmarkX = actualLandmarks[j].x;
          landmarkY = actualLandmarks[j].y;
          break;
        }
      }
      
      // Calculations
      double term_1, term_2, denominator, weight;
      term_1 = (observedX - landmarkX) * (observedX - landmarkX) / (2 * std_x * std_x);
      term_2 = (observedY - landmarkY) * (observedY - landmarkY) / (2 * std_y * std_y);
      denominator = 1 / (2 * M_PI * std_x * std_y);
      
      // Assign value
      weight = denominator * std :: exp(-1 * (term_1 + term_2));
      if (weight == 0){
        particles[p].weight *= 1e-5;
      }
      else {
        particles[p].weight *= weight;
      }
    }
  }
}

void ParticleFilter::resample() {  
  // Inititalize random generator
  std :: default_random_engine gen;
  
  // Assign weight vector
  for (int i = 0; i < num_particles; i += 1){
    weights[i] = particles[i].weight;
  }
  
  // Generate distributions
  std :: uniform_real_distribution<double> distW(0.0, *max_element(weights.begin(), weights.end()));
  std :: uniform_int_distribution<int> distI(0, num_particles - 1);
  
  // Resample using the Wheel method
  int index = distI(gen);
  double weight_sum = 0.0;
  
  vector<Particle> resampled;
  for(int i = 0; i < num_particles; i += 1){
    weight_sum += distW(gen) * 2.0;
    
    while (weight_sum > weights[index]){
      weight_sum -= weights[index];
      index = (index + 1) % num_particles;
    }
    
    resampled.push_back(particles[index]);
  }
  
  // Assign the particles
  particles = resampled;
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