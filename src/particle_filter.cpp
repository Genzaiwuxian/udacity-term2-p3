/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (!is_initialized)
	{
		//set total particles number
		num_particles = 60;

		default_random_engine gen;

		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);

		Particle particle;

		//pass all x y theta and weigts to each(num_particles) particles
		for (unsigned int i = 0; i < num_particles; ++i)
		{
			particle.id = i;
			particle.x = dist_x(gen);
			particle.y = dist_y(gen);
			particle.theta = dist_theta(gen);
			particle.weight = 1.0;
			particles.push_back(particle);
		}

		is_initialized = true;
	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	for (decltype(particles.size()) i=0;i<particles.size();++i )
	{
		double theta_p = particles[i].theta;

		if (fabs(yaw_rate) > 1.0E-8)
		{
			particles[i].x = particles[i].x + velocity / yaw_rate * (sin(theta_p) + yaw_rate * delta_t - sin(theta_p));
			particles[i].y = particles[i].y + velocity / yaw_rate * (cos(theta_p) - cos(theta_p + yaw_rate * delta_t));
		}
		else
		{
			particles[i].x = particles[i].x + velocity * cos(theta_p)*delta_t;
			particles[i].y = particles[i].y + velocity * sin(theta_p)*delta_t;
		}
		particles[i].theta = particles[i].theta + yaw_rate * delta_t;

		//add random Gaussian noise
		default_random_engine gen;
		normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	if (predicted.size() != 0 && observations.size() != 0)
	{
		for (decltype(observations.size()) j = 0; j < observations.size(); ++j)
		{
			int obs_id = observations[j].id;
			double obs_x = observations[j].x;
			double obs_y = observations[j].y;

			double best_distance = 10000.0;
			int best_id = -1;

			for (decltype(predicted.size()) i = 0; i < predicted.size(); ++i)
			{
				int p_id = predicted[i].id;
				double p_x = predicted[i].x;
				double p_y = predicted[i].y;

				double distance_diff = dist(p_x, p_y, obs_x, obs_y);
				if (distance_diff < best_distance)
				{
					best_distance = distance_diff;
					best_id = p_id;
				}
			}

			observations[j].id = best_id;

		}
	}
	else
	{
		std::cout << "predicted or observations size =0" << std::endl;
	}
	
	//calculate nearest neighbor
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double std_x = std_landmark[0];
	double std_y = std_landmark[1];

	double std_x2 = std_x * std_x;
	double std_y2 = std_y * std_y;

	vector<double> weights_sum;

	for (unsigned int i=0;i<num_particles;++i)
	{
		int p_id = particles[i].id;
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;
		
		//transform_observations
		vector<LandmarkObs> observations_transform;
		for (decltype(observations.size()) j = 0; j < observations.size(); ++j)
		{
			int observations_tran_id = observations[j].id;
			double observations_tran_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
			double observations_tran_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
			observations_transform.push_back(LandmarkObs{ observations_tran_id, observations_tran_x, observations_tran_y });
		}

		vector<LandmarkObs> predictions;
		for (decltype(map_landmarks.landmark_list.size()) j = 0; j < map_landmarks.landmark_list.size(); ++j)
		{
			int map_id = map_landmarks.landmark_list[j].id_i;
			double map_x = map_landmarks.landmark_list[j].x_f;
			double map_y = map_landmarks.landmark_list[j].y_f;

			double diff_x = p_x - map_x;
			double diff_y = p_y - map_y;

			if (fabs(diff_x)<sensor_range && fabs(diff_y)<sensor_range)
			{
				predictions.push_back(LandmarkObs{ map_id, map_x, map_y });
			}
		}

		//association
		ParticleFilter::dataAssociation(predictions, observations_transform);

		/*
		
		for (unsigned int k = 0; k < predictions.size(); ++k)
		{
			cout << "prediction: " << k << endl;
			cout << predictions[k].id << endl;
			cout << predictions[k].x << endl;
			cout << predictions[k].y << endl;
		}

		for (unsigned int j = 0; j < observations_transform.size(); ++j)
		{
			cout << "observations_transform: " << j << endl;
			cout << observations_transform[j].id << endl;
			cout << observations_transform[j].x << endl;
			cout << observations_transform[j].y << endl;
		}
		*/
		


		//update weights
		double weights_diff = 1.0;
		for (decltype(predictions.size()) i = 0; i < predictions.size(); ++i)
		{
			for (decltype(observations_transform.size()) j = 0; j < observations_transform.size(); ++j)
			{
				
				if (observations_transform[j].id == predictions[i].id)
				{
					double diff_x = observations_transform[j].x - predictions[i].x;
					double diff_x2 = diff_x * diff_x;

					double diff_y = observations_transform[j].y - predictions[i].y;
					double diff_y2 = diff_y * diff_y;

					double gauss_norm = (1 / (2 * M_PI*std_x*std_y));
					double exponent = ((diff_x2) / (2 * std_x2) + (diff_y2) / (2 * std_y2));
					// cout << "gauss_norm: " << gauss_norm << endl;
					// cout << "exponent: " << exponent << endl;
					double weight_temp = gauss_norm * exp(-exponent);
					// cout << "weight_temp: " << weight_temp << endl;
					weights_diff *= weight_temp;
				} 
			}
		}
		cout<< "weights_diff: " << weights_diff << endl;
		particles[i].weight = weights_diff;
		weights_sum.push_back(weights_diff);
		// cout << "particle: " << i << " weight is " << particles[i].weight << endl;
	}
	weights = weights_sum;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<int> distribution(0, num_particles);
	auto index = distribution(gen);

	auto max_weight = *max_element(weights.begin(), weights.end());

	double beta = 0.0;

	uniform_real_distribution<double> weight_distribution(0.0, max_weight);
	
	vector<Particle> particles_resample;
	for (unsigned int i = 0; i < num_particles; ++i)
	{
		beta += 2.0*weight_distribution(gen);
		while (particles[index].weight < beta)
		{
			beta -= particles[index].weight;
			index=(index+1)%num_particles;
		}
		particles_resample.push_back(particles[index]);
	}

	particles = particles_resample;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
