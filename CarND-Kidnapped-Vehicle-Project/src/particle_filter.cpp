#include "particle_filter.h"
#include "helper_functions.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  Particle particle;
  //set the number of particles. 
  num_particles = 400; //todo: fine tune number of particles for better performance
  default_random_engine generator;
  //random number distribution, produces values according to normal distribution
  normal_distribution<double> dist_x(x, std[0]); //gps_x, std_x
  normal_distribution<double> dist_y(y, std[1]); //gps_y, std_y
  normal_distribution<double> dist_theta(theta, std[2]); //theta, std_theta  

  for (int i = 0; i < num_particles; i++){  
    particle.id = i;
    //initialize all particles to first position, add random gaussian noise to each particle.
    particle.x = dist_x(generator);
    particle.y = dist_y(generator);
    particle.theta = dist_theta(generator);
    //all weights to 1
    particle.weight = 1.0; 
    particles.push_back(particle);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {  
  double theta;                                
  default_random_engine generator;
  normal_distribution<double> dist_x(0, std_pos[0]); // std x
  normal_distribution<double> dist_y(0, std_pos[1]); // std y
  normal_distribution<double> dist_theta(0, std_pos[2]); // std theta

  for (int i = 0; i < num_particles; i++) {
  	theta = particles[i].theta;

    if (abs(yaw_rate) < 0.001) { //first formula in Lesson3_4(yaw rate and velocity)
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
      particles[i].theta = theta; 
    } 
    else { //second formula in Lesson3_4(yaw rate and velocity)
      particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;      
    }

    //add uncertainity
    particles[i].x += dist_x(generator);
    particles[i].y += dist_y(generator);
    particles[i].theta += dist_theta(generator);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  double minDistance, distance;
  for (int i = 0; i < (int)observations.size(); i++) {
    minDistance = numeric_limits<double>::max();
    for (int j = 0; j < (int)predicted.size(); j++) {
      distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (distance < minDistance) {
        minDistance = distance;
        observations[i].id = predicted[j].id; //associate transformed observation with nearest landmark so far (with id)
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {  
  double theta;
  double weight;
  double transformed_x, transformed_y; //todo: may not be used

  for (int i = 0; i < num_particles; i++) {
    theta = particles[i].theta;  
    //get landmarks that are in sensor range
    vector<LandmarkObs> landmarksInRange;
    for(int j = 0; j < (int)map_landmarks.landmark_list.size(); j++) {
      if(dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f) <= sensor_range)
        landmarksInRange.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f});
    }
    
    //transform noisy observations into map coordinate system
    vector<LandmarkObs> transformedObservations;
    for(int j = 0; j < (int)observations.size(); j++) {
      //transform noisy observations into map coordinate system, (homogenous transformation)
      transformed_x = cos(theta)*observations[j].x - sin(theta)*observations[j].y + particles[i].x;
      transformed_y = sin(theta)*observations[j].x + cos(theta)*observations[j].y + particles[i].y;
      transformedObservations.push_back(LandmarkObs{observations[j].id, transformed_x, transformed_y});
    }       
    //associate transformed noisy observations with nearest landmark by setting its id as the nearest landmark id
    dataAssociation(landmarksInRange, transformedObservations);

    particles[i].weight = 1.0; //clear weight, total weight will be found by multiplying all the weights
    for(int j = 0; j < (int)transformedObservations.size(); j++) {
      for(int k = 0; k < (int)landmarksInRange.size(); k++) {
        if (landmarksInRange[k].id == transformedObservations[j].id) {
          //multivariate gaussian probability
          weight = multiv_prob(std_landmark[0], std_landmark[1],                 //standard deviation x-y
                               transformedObservations[j].x, transformedObservations[j].y, //observation x-y
                               landmarksInRange[k].x, landmarksInRange[k].y);    //associated landmark x-y
          break;
        }
      }

      if (weight == 0)
        particles[i].weight = 0.0;
      else
        //total weight of the particle 
        particles[i].weight *= weight;  
    }
  }
}

void ParticleFilter::resample() {
  vector<double> weights;
  vector<Particle> resampled;
  double maxWeight, beta = 0.0;
  int index;
  default_random_engine generator;
  uniform_int_distribution<int> uniformIntDistribution(0, num_particles - 1);
  
  //store particle weights
  for ( vector<Particle>::iterator it = particles.begin(); it!= particles.end(); ++it )
  	weights.push_back((*it).weight);    
  
  maxWeight = *max_element(weights.begin(), weights.end()); //get max weight       
  uniform_real_distribution<double> uniformDoubleDistribution(0.0, maxWeight);
  
  index = uniformIntDistribution(generator); //get a particle randomly
  //resampling wheel implementation
  for(int i = 0; i < num_particles; i++) {
    beta += uniformDoubleDistribution(generator) * 2.0; 
    while(beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled.push_back(particles[index]);
  }
  particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);
  return s;
}