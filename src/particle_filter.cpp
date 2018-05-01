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
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (!is_initialized)
	{
		//set total particles number
		num_particles = 10;

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

			//cout << "particle id " << particles[i].id << endl;
			//cout << "particle x " << particles[i].x << endl;
			//cout << "particle y " << particles[i].y << endl;
			//cout << "particle theta" << particles[i].theta << endl;
			//cout << "particle weight " << particles[i].weight << endl;
		}

		SIR = false;
		SysR = true;

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

		if (fabs(yaw_rate) > 0.00001)
		{
			particles[i].x = particles[i].x + velocity / yaw_rate * (sin((theta_p) + yaw_rate * delta_t) - sin(theta_p));
			particles[i].y = particles[i].y + velocity / yaw_rate * (cos(theta_p) - cos(theta_p + yaw_rate * delta_t));
		}
		else
		{
			particles[i].x = particles[i].x + velocity * cos(theta_p)*delta_t;
			particles[i].y = particles[i].y + velocity * sin(theta_p)*delta_t;
		}
		particles[i].theta = particles[i].theta + yaw_rate * delta_t;

		//add random Gaussian noise
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

			double best_distance = 100000.0;
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
		
		//update weights
		double weights_diff = 1.0;
		for (decltype(observations_transform.size()) m = 0; m < observations_transform.size(); ++m)
		{
			for (decltype(predictions.size()) n = 0; n < predictions.size(); ++n)
			{
				if (observations_transform[m].id == predictions[n].id)
				{
					double diff_x = observations_transform[m].x - predictions[n].x;
					double diff_x2 = diff_x * diff_x;

					double diff_y = observations_transform[m].y - predictions[n].y;
					double diff_y2 = diff_y * diff_y;
					// cout << "diff_x2 " << diff_x2 << endl;
					// cout << "diff_y2 " << diff_y2 << endl;

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
		// cout<< "weights_diff: " << weights_diff << endl;
		particles[i].weight = weights_diff;
		// cout << "particle: " << i << " weight is " << particles[i].weight << endl;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	double total_weights = 0;
	vector<double> weights_sum;
	for (unsigned int i = 0; i < num_particles; ++i)
	{
		weights_sum.push_back(particles[i].weight);
		total_weights += particles[i].weight;
	}

	for (unsigned int i = 0; i < num_particles; ++i)
		weights_sum[i] /= total_weights;

	//calculate accumulated weights, such as weights_accu[0]=paticles[0].weight, weights_accu[1]=particles[0]+[1].weights
	vector<double> weights_accu;
	total_weights = 0;
	for (unsigned int i = 0; i < num_particles; ++i)
	{
		total_weights += weights_sum[i];
		weights_accu.push_back(total_weights);
	}

	//Sample importance resampling algorthium
	if (SIR)
	{
		discrete_distribution<int> distribution(0, num_particles);
		auto index = distribution(gen);

		auto max_weight = *max_element(weights_sum.begin(), weights_sum.end());

		double beta = 0.0;

		uniform_real_distribution<double> weight_distribution(0.0, max_weight);

		vector<Particle> particles_resample;
		for (unsigned int i = 0; i < num_particles; ++i)
		{
			beta += 2.0*weight_distribution(gen);
			while (weights_sum[index] < beta)
			{
				beta -= weights_sum[index];
				index = (index + 1) % num_particles;
			}
			particles_resample.push_back(particles[index]);
		}

		particles = particles_resample;
	}
	
	if (SysR)
	{
		uniform_real_distribution<double> distribution(0.0, 1.0);
		auto ran_discrete = distribution(gen);

		vector<double> weights_random;
		for (unsigned int i = 0; i < num_particles; ++i)
		{
			weights_random.push_back((ran_discrete + i) / num_particles);
		}

		uniform_int_distribution<int> distribution_index(0, num_particles-1);
		auto index = distribution_index(gen);

		vector<Particle> particles_resample;
		for (unsigned int i = 0; i < num_particles; ++i)
		{
			while (weights_accu[index] < weights_random[i])
			{
				(++index) % num_particles;
			}
			particles_resample.push_back(particles[index]);
		}
		particles = particles_resample;
	}


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
