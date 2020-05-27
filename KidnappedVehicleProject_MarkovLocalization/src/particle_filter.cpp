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


void ParticleFilter::init( double x, double y, double theta, double std[]) {

	// TODO: Set the number of particles. - DONE.  Initialize all particles to first position (based on estimates of
	// TODO: x, y, theta and their uncertainties from GPS) and all weights to 1. - DONE
	// Add random Gaussian noise to each particle. - DONE
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file). - Done.

    num_particles  = 100;
    default_random_engine gen;

    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

	for(int i=0;i<num_particles;i++){

        Particle particle_i;
        particle_i.id = i+1;
        particle_i.x  = dist_x(gen);
        particle_i.y = dist_y(gen);
        particle_i.theta = dist_theta(gen);
        particle_i.weight = 1;

        weights.push_back(1);
        particles.push_back(particle_i);
    }
    // The above part is done.
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // Note: Standard deviation of the measurements is chosen for the standard deviation of the prediction step.
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];
    default_random_engine gen;

    for (int i=0;i<num_particles;i++){

        Particle particle_i = particles[i];

        if(yaw_rate==0){
            particle_i.x = particle_i.x + velocity*delta_t*cos(particle_i.theta);
            particle_i.y = particle_i.y + velocity*delta_t*sin(particle_i.theta);
        }else{
            particle_i.x = particle_i.x + (velocity/yaw_rate)*(sin(particle_i.theta + yaw_rate*delta_t) - sin(particle_i.theta));
            particle_i.y = particle_i.y + (velocity/yaw_rate)*(cos(particle_i.theta) - cos(particle_i.theta + yaw_rate*delta_t));
            particle_i.theta = particle_i.theta + yaw_rate*delta_t;
        }

        normal_distribution<double> dist_x(particle_i.x, std_x);
        normal_distribution<double> dist_y(particle_i.y, std_y);
        normal_distribution<double> dist_theta(particle_i.theta, std_theta);

        particle_i.x = dist_x(gen);
        particle_i.y = dist_y(gen);
        particle_i.theta = dist_theta(gen);

        particles[i] = particle_i;

        // Clearing all the redundant values from the particle class.
        particles[i].sense_x.clear();
        particles[i].sense_y.clear();
        particles[i].associations.clear();
        particles[i].coordinates.clear();

    }
}

// predicted: Prediction measurements between one particular particle and all of the map landmarks within the sensor range.
// observations: Actual landmark measurements gathered from LIDAR.

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    double minimum_value = -1.0;
    int point_id;

    for (int i=0;i<observations.size() ;i++){
        for(int j=0;j<predicted.size();j++){

            double square_sum = sqrt(pow(predicted[j].x - observations[i].x,2.0)+pow(predicted[j].y - observations[i].y,2));
            if(minimum_value<0){
                minimum_value = square_sum;
                point_id = j;

            }else if(square_sum<minimum_value){
                minimum_value = square_sum;
                point_id = j;
            }
        }
        observations[i].id = point_id;
    }
}


//
//vector<float> ParticleFilter::getValues(const Map &map_landmarks, int id){
//
//    vector<float> values;
//
//    for (int i=0;i<map_landmarks.landmark_list.size();i++){
//        if(map_landmarks.landmark_list[i].id_i==id){
//            values.push_back(map_landmarks.landmark_list[i].x_f);
//            values.push_back(map_landmarks.landmark_list[i].y_f);
//        }
//        break;
//    }
//    return values;
//}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    // Need to use the standard deviation of the landmark.
    //    normal_distribution<double> dist_x(x, std_x);

    vector<vector<float>> coordinates;

    for( int i=0; i<num_particles; i++){

        Particle present_particle = particles[i];

        for(int j=0;j<observations.size();j++){

            LandmarkObs presentObs = observations[j];
            double new_x = particles[i].x + cos(particles[i].theta)*presentObs.x - sin(particles[i].theta)*presentObs.y;
            double new_y = particles[i].y + sin(particles[i].theta)*presentObs.x + cos(particles[i].theta)*presentObs.y;

            particles[i].sense_x.push_back(new_x);
            particles[i].sense_y.push_back(new_y);

            double minimum_value = -1.0;
            int id;
            vector<float> co(2);

            for( int k = 0; k < map_landmarks.landmark_list.size(); k++){

                double dist_diff =sqrt(pow(particles[i].x - map_landmarks.landmark_list[k].x_f,2)+pow(particles[i].y - map_landmarks.landmark_list[k].y_f,2));
                if(dist_diff<sensor_range){

                    double dist_diff2 = sqrt(pow(new_x - map_landmarks.landmark_list[k].x_f,2)+pow(new_y - map_landmarks.landmark_list[k].y_f,2));
                    if(minimum_value<0){
                        minimum_value = dist_diff2;
                        id = map_landmarks.landmark_list[k].id_i;
                        co[0] = map_landmarks.landmark_list[k].x_f;
                        co[1] = map_landmarks.landmark_list[k].y_f;
                    }
                    else if(dist_diff2<minimum_value){
                        minimum_value = dist_diff2;
                        id = map_landmarks.landmark_list[k].id_i;
                        co[0] = map_landmarks.landmark_list[k].x_f;
                        co[1] = map_landmarks.landmark_list[k].y_f;
                    }
                }
            }
            particles[i].associations.push_back(id);
            particles[i].coordinates.push_back(co);
        }
    }

    double obs_std_x = std_landmark[0];
    double obs_std_y = std_landmark[1];

    // Now I have to calculate the weights of all the particles.
    for (int i=0;i<num_particles;i++){

        Particle present_particle = particles[i];
        double probability  = 1.0;

        for(int j=0;j<present_particle.associations.size();j++){

            vector<float> coords = present_particle.coordinates[j];
            double p_x = present_particle.sense_x[j];
            double p_y = present_particle.sense_y[j];
            probability = probability*(1./(2*M_PI*obs_std_x*obs_std_y))*exp(-(pow(p_x - coords[0],2)/(2*pow(obs_std_x,2)))-(pow(p_y-coords[1],2)/(2*pow(obs_std_y,2))));
        }
        particles[i].weight = probability;
        weights[i] = probability;
    }

    // Finally need to normalize these weight, to bring them in the range [0,1].
    double weight_sum = 0.0;
    for (int i=0;i<num_particles;i++){
        weight_sum += particles[i].weight;
    }

    for(int i=0;i<num_particles;i++){
        particles[i].weight = particles[i].weight/weight_sum;
        weights[i] = particles[i].weight;
    }

}

void ParticleFilter::resample() {

	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    vector<Particle> newParticles;
    default_random_engine gen;

    double maximum_weight_value = 0.0;
    for (int i=0;i<num_particles;i++){
        if(weights[i]>maximum_weight_value){
            maximum_weight_value = weights[i];
        }
    }

    for (int i=0;i<num_particles;i++){
        weights[i] = weights[i]/maximum_weight_value;
    }

//    discrete_distribution<> distribution(weights.begin(),weights.end());
//    int number = distribution(gen);

    double beta = 0;

    uniform_real_distribution<double> uniRealDist(0.0, 2*maximum_weight_value);
    uniform_int_distribution<int>     uniIntDist(0, num_particles-1);

    int index = uniIntDist(gen);

    for (int i=0;i<num_particles;i++){
        beta = beta + uniRealDist(gen);

        while(weights[index]<beta){
            beta = beta - weights[index];
            index = index + 1;
            if(index>num_particles-1){
                index = 0;
            }
        }
        newParticles.push_back(particles[index]);
    }

    particles = newParticles;

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

    return particle;
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
