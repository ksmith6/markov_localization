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

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;

	std::default_random_engine gen;
	std::normal_distribution<double> N_x(x, std[0]);
	std::normal_distribution<double> N_y(y, std[1]);
	std::normal_distribution<double> N_theta(theta, std[2]);

	// Initialize particles.
	for (int i=0; i<num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1.0;
		particles.push_back(particle);
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// EPSILON defines threshold between normal dynamics and zero yaw-rate dynamics.
	double EPSILON = 0.000001; // rad / s

	std::default_random_engine gen;
	std::normal_distribution<double> N_x(0.0, std_pos[0]);
	std::normal_distribution<double> N_y(0.0, std_pos[1]);
	std::normal_distribution<double> N_theta(0.0, std_pos[2]);

	// Advance each state according the bicycle model while injecting process noise into it.
	for (int i=0; i<num_particles; i++) {
		
		// Create shorthand variable for the i^th particle
		Particle p = particles[i];

		// Instantiate the variables for updated states.
		double xf;
		double yf;
		double thetaf;

		// Switch dynamics based on turn rate.
		if (fabs(yaw_rate) > EPSILON) {
			// Standard bicycle model dynamics

			// Cache product to avoid repeated multiplications
			double thetaDot_dt = yaw_rate*delta_t;

			xf = p.x + velocity/yaw_rate * (sin(p.theta + thetaDot_dt) - sin(p.theta));
			yf = p.y + velocity/yaw_rate * (cos(p.theta) - cos(p.theta + thetaDot_dt));
			thetaf = p.theta + thetaDot_dt;
		} else {
			// Zero Yaw-Rate Dynamics
			xf = p.x + velocity * delta_t * cos(p.theta);
			yf = p.y + velocity * delta_t * sin(p.theta);
			thetaf = p.theta;
		}
		
		// Overwrite particle state in particles vector with deterministic state + noise.
		particles[i].x = xf + N_x(gen);
		particles[i].y = yf + N_y(gen);
		particles[i].theta = thetaf + N_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//for (int i=0; i<observations.size(); i++) {

	//}

	// Transform each observation from vehicle-relative coordinates into map coordinates
	// z_global = x_particle + T(-theta) * [z_x; z_y]

	// Compute distance between this observation and every landmap in the map.
	//   If the distance is smaller than the previous distance, then store that landmark as the associated landmark.

	// 


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	
	// Maintain running sum of weights for normalization.
	double sumW = 0;

	for (int i=0; i<num_particles; i++) {
		double w = 1.0;

		std::vector<Map::single_landmark_s> close_landmarks;
		for (int j=0; j<map_landmarks.landmark_list.size(); j++) {
			if (dist(map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f, particles[i].x, particles[i].y) <= sensor_range) {
				close_landmarks.push_back(map_landmarks.landmark_list[j]);
			}
		}


		// Cache expensive trig operations.
		double cosH = cos(-particles[i].theta);
		double sinH = sin(-particles[i].theta);

		for (int j=0; j<observations.size();j++) {
			// Transform the observations into map coordinate system.
			//    z_m = xp + T * z_v
			double ox = particles[i].x + cosH * observations[j].x + sinH * observations[j].y;
			double oy = particles[i].y - sinH * observations[j].x + cosH * observations[j].y;

			// Perform landmark-observation association with nearest-neighbor search.
			double minDist;
			double associated_landmark_x;
			double associated_landmark_y;
			for (int k=0; k<map_landmarks.landmark_list.size(); k++) {
				double thisDist = dist(ox, oy, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
				
				// If this is the first iteration or if the distance is less than the minimum distance, 
				if (k==0 || (thisDist < minDist)) {
					// Then update the minimum distance.
					minDist = thisDist;
					associated_landmark_x = map_landmarks.landmark_list[k].x_f;
					associated_landmark_y = map_landmarks.landmark_list[k].y_f;
				}
			} // end of nearest neighbor search loop

			// Now we have the closest landmark to the current observation.  
			// Let's condition the weight by its likelihood.

			// Let's evaluate the multivariate Gaussian PDF
			double firstTerm = 1.0 / (2 * 3.14159265358979323846 * std_landmark[0] * std_landmark[1]);
			double xComp = 0.5 * (ox - associated_landmark_x)*(ox - associated_landmark_x) / (std_landmark[0]*std_landmark[0]);
			double yComp = 0.5 * (oy - associated_landmark_y)*(oy - associated_landmark_y) / (std_landmark[1]*std_landmark[1]);;
			double pdf = firstTerm * exp(-(xComp + yComp)); 

			particles[i].weight *= pdf;

		} // end of observation processing loop
		sumW += particles[i].weight;
	} // end of particle loop

	// Normalization step
	for (int i=0; i < num_particles; i++) {
		particles[i].weight /= sumW;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
