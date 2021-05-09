/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang -> editted by Donghan Kim
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

// checked
void ParticleFilter::init(double x, double y, double theta, double std[]) {

    num_particles = 100;
	weights.resize(num_particles);
	particles.resize(num_particles);
	std::random_device rn;
	std::default_random_engine rn_gen(rn());

	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	for(int i = 0; i < num_particles; i++){
		particles[i].id = i;
		particles[i].x = dist_x(rn_gen);
		particles[i].y = dist_y(rn_gen);
		particles[i].theta = dist_theta(rn_gen);
		particles[i].weight = 1.0;
	}

	is_initialized = true;
}

// checked
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate){

	std::default_random_engine rn_gen;
	std::normal_distribution<double> dist_x(0, std_pos[0]);
	std::normal_distribution<double> dist_y(0, std_pos[1]);
	std::normal_distribution<double> dist_theta(0, std_pos[2]);

	for(int i = 0; i < num_particles; i++){
		// try < 0
		if(fabs(yaw_rate) < 0.000001){
			particles[i].x += velocity*delta_t*cos(particles[i].theta);
			particles[i].y += velocity*delta_t*sin(particles[i].theta);
		} else{
			particles[i].x += velocity/yaw_rate*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y += velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		particles[i].x += dist_x(rn_gen);
		particles[i].y += dist_y(rn_gen);
		particles[i].theta += dist_theta(rn_gen);
	}
}

// checked
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {

	for(int i = 0; i < observations.size(); i++){
		LandmarkObs curr_obs = observations[i];
		double min_dist = INFINITY;
		int closest_particle_id = -1;

		for(int j = 0; j < predicted.size(); j++){
			LandmarkObs curr_pred = predicted[j];
			double curr_dist = dist(curr_obs.x, curr_obs.y, curr_pred.x, curr_pred.y);

			if(curr_dist <= min_dist){
				min_dist = curr_dist;
				closest_particle_id = j;
			}
		}
		observations[i].id = closest_particle_id;
	}
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations, const Map &map_landmarks) {
	auto cov_x = pow(std_landmark[0], 2.0);
	auto cov_y = pow(std_landmark[1], 2.0);
	auto normalizer = 2.0*M_PI*std_landmark[0]*std_landmark[1];

	for(int i = 0; i < particles.size(); i++){
		double px = particles[i].x;
		double py = particles[i].y;
		double ptheta = particles[i].theta;

		std::vector<LandmarkObs> predictions;
		for(int j = 0; j < map_landmarks.landmark_list.size(); j++){
			float lmx = map_landmarks.landmark_list[j].x_f;
			float lmy = map_landmarks.landmark_list[j].y_f;
			int lm_id = map_landmarks.landmark_list[j].id_i;

			if(dist(lmx, lmy, px, py) <= sensor_range){
				LandmarkObs new_particle;
				new_particle.x = lmx;
				new_particle.y = lmy;
				new_particle.id = lm_id;
				predictions.push_back(new_particle);
			}
		}
		if(predictions.size() == 0){
			std::cout << "predictions size = 0..." << std::endl;
			particles[i].weight = 0;
			weights[i] = 0;
		} else{
			std::vector<LandmarkObs> transformed;
			// transform into map space
			for(int k = 0; k < observations.size(); k++){
				double tx = cos(ptheta)*observations[k].x-sin(ptheta)*observations[k].y+px;
				double ty = sin(ptheta)*observations[k].x+cos(ptheta)*observations[k].y+py;
				LandmarkObs new_transformed;
				new_transformed.x = tx;
				new_transformed.y = ty;
				new_transformed.id = observations[k].id;
				transformed.push_back(new_transformed);
			}

			dataAssociation(predictions, transformed);

			// multi-variate Gaussian calculation
			double prob = 1.0;
			for(int l = 0; l < transformed.size(); l++){
				LandmarkObs trans_obj = transformed[l];
				LandmarkObs pred_obj = predictions[trans_obj.id];

				auto dx = trans_obj.x - pred_obj.x;
				auto dy = trans_obj.y - pred_obj.y;
				prob *= exp(-(dx * dx / (2 * cov_x) + dy * dy / (2 * cov_y))) / normalizer;
			}
			/*
			if(prob == 0){
				particles[i].weight = 0.000001;
				weights[i] = 0.000001;
			} else{
				particles[i].weight = prob;
				weights[i] = prob;
			}
			*/
			particles[i].weight = prob;
			weights[i] = prob;
		}
	}
}

void ParticleFilter::resample() {
	std::random_device rn;
	std::default_random_engine rn_gen(rn());
	std::discrete_distribution<int> dist_index(weights.begin(), weights.end());
	std::vector<Particle> resampled_particles(particles.size());

	for(int i = 0; i < particles.size(); i++){
		resampled_particles[i] = particles[dist_index(rn_gen)];
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
