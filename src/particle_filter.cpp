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

		// TODO - Compute likelihood of each measurements
		
		//double denominator = sqrt(2*pi*sigma);
		//for (int j=0; j < num_measurements; j++) {
		//	w *= denominator * exp(-0.5 * dxT * sigInv * dx);
		//}
		
		sumW += w;

		particles[i].weight = w;
	}

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
