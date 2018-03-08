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

	//set num of particles
	num_particles=100;

	//random engine
	default_random_engine gen;

	//set std deviation given by std[]
	double std_x = std[0];
	double std_y = std[1];;
	double std_theta =std[2];

	//creates a normal (Gaussian) distribution for x, y and theta.
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	//run thru all the particles, add Gaussian noise and set weight=1
	for(int i=0; i<num_particles; i++)
	{
			Particle particle;
			particle.id = i;
			particle.x = x;
			particle.y = y;
			particle.theta = theta;
			particle.weight = 1.0;

			//add noise
			particle.x += dist_x(gen);
			particle.y += dist_y(gen);
			particle.theta += dist_theta(gen);

			particles.push_back(particle);
	}

	weights = vector<double>(num_particles, 1.0);

	cout <<particles.size()<< " ----particles have been created---- " << "\n";

	}
	//initializing done
	is_initialized = true;

}//end ParticleFilter::init


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	//random engine
	default_random_engine gen;

	double std_x_pos = std_pos[0];
	double std_y_pos = std_pos[1];;
	double std_theta_pos =std_pos[2];

	for(int i=0; i<num_particles; i++)
	{

		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		//creates a normal (Gaussian) distribution for all particles x, y and theta
		normal_distribution<double> dist_x(0, std_x_pos);
		normal_distribution<double> dist_y(0, std_y_pos);
		normal_distribution<double> dist_theta(0, std_theta_pos);

		//new state
		if(abs(yaw_rate) > 0.00001)
		{
		particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate*delta_t) - sin(theta));
		particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate*delta_t));
		particles[i].theta += yaw_rate * delta_t;
		}

		else
		{
		particles[i].x += velocity * delta_t * cos(theta);
		particles[i].y += velocity * delta_t * sin(theta);
		}

		//add noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);

	}
}//end ParticleFilter::prediction


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	int num_predictions = predicted.size();
	int num_observations = observations.size();
	int index;
	double min_dist;
	double d;

	for(int i=0; i<num_observations; i++)
	{
			min_dist=10000; //initialize min_dist
			int min_id=-1; //initialize min_id

			for(int j=0; j<num_predictions; j++)
			{
				//calculate the distance between the predicted measurement to each observed measurement
				d = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
				//std::cout <<"Dist:"<<d<<std::endl;
				if (d < min_dist)
						{
						min_dist = d;
						index = j;
						observations[i].id = predicted[index].id;
						}
			}
			//std::cout <<"Landmark Index:"<<observations[i].id <<std::endl;
			//std::cout <<"transformed Observation(x,y): "<<"("<<observations[i].x<<","<<observations[i].y<<") ;";
			//std::cout <<"Predicted(x,y): "<<"("<<predicted[index].x<<","<<predicted[index].y<<")"<<"Min_Dist:"<<min_dist<<std::endl;


	}
}// end ParticleFilter::dataAssociation


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks){
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

	int num_landmarks = map_landmarks.landmark_list.size();
	int num_observations = observations.size();
	double std_x_lm = std_landmark[0];
	double std_y_lm = std_landmark[1];
	double d;

	for(int i=0; i<num_particles; i++)
	{

			double x = particles[i].x;
			double y = particles[i].y;
			double theta = particles[i].theta;

			//create a vector with the landmarks within sensor range of the particle
			vector<LandmarkObs> predicted_map;

			for(int j=0; j<num_landmarks; j++)
			{
					double lm_x = map_landmarks.landmark_list[j].x_f;
					double lm_y = map_landmarks.landmark_list[j].y_f;
					int lm_id = map_landmarks.landmark_list[j].id_i;

					//calculate the distance between particle and predicted landmarks
					d = dist(x, y, lm_x, lm_y);
					if (d < sensor_range)
					{
							//list of predicted landmarks in sensor range
							predicted_map.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
					}
			}

			//convert observations to map coordinate system
			std::vector<LandmarkObs> observations_map;

			for (int j=0; j<num_observations; j++)
			{
					double x_obs = observations[j].x;
					double y_obs = observations[j].y;
					int id_obs = observations[j].id;

					//calculate the transformation from vehicle to map coordinate system
					double x_obs_trans = x + (cos(theta) * x_obs) - (sin(theta) * y_obs);
					double y_obs_trans = y + (sin(theta) * x_obs) + (cos(theta) * y_obs);

					//list of observations (map coordinate system)
					observations_map.push_back(LandmarkObs{ id_obs, x_obs_trans, y_obs_trans });
			}

			//perform dataAssociation for the observations_map und predicted_map
			dataAssociation(predicted_map, observations_map);

			//reinit weight
			particles[i].weight = 1.0;

			//Define the assocated prediction and observation input for weight calculation by Multivariate Gaussian
			double obs_x;
			double obs_y;
			int assoc_id;
			double pre_x;
			double pre_y;

			if (predicted_map.size() == observations_map.size())
			{

				for (int j=0; j<observations_map.size(); j++)
				{

					obs_x = observations_map[j].x;
					obs_y = observations_map[j].y;
					assoc_id = observations_map[j].id;

					for (int k=0; k<predicted_map.size(); k++)
					{
						if (assoc_id == predicted_map[k].id)
						{
							pre_x = predicted_map[k].x;
							pre_y = predicted_map[k].y;
						}
					}

					//Calculate weight for each association
					double gauss_norm = (1/(2 * M_PI * std_x_lm * std_y_lm));
					double exponent = pow((pre_x - obs_x),2.0)/(2.0 * pow((std_x_lm),2.0)) + pow((pre_y - obs_y),2.0)/(2.0 * pow(std_y_lm,2.0));
					double weight = gauss_norm * exp(-1.0 * exponent);

					//Calculate total weight and update weight vector for resample task
					particles[i].weight = particles[i].weight * weight;
					weights[i]=particles[i].weight;
				}
			}
			else
			{
					particles[i].weight = 0;
					weights[i]=particles[i].weight;
			}
	}

}//end ParticleFilter::updateWeights

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


	//Code Source -> Q&A for Kidnapped Vehicle: https://www.youtube.com/watch?v=-3HI3Iw3Z9g&feature=youtu.be
	//random engine
	default_random_engine gen;

	//create distribution with weights as normalization values
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resampled_particles;

	for(int i=0; i<num_particles; i++)
	{
		resampled_particles.push_back(particles[distribution(gen)]);
	}
	particles = resampled_particles;

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
