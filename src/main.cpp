/*
 * Description: This code to try kalman filtering of points measured through a noisy sensor,
 *              and needs to be corrected by some means. A simplest approach which everyone at first
 *              for is averaging (simple low pass filter). A second tool which comes handy is a 
 *              Kalman Filter approach. 
 *              Experiments can be tried out by tweaking, delta_t, initial_velocity, acc_var, 
 *              and meas_var
 * 
 * Note: This code do not come with warranty of any kind, please don't rely on this code if its 
 *       utilization involves risk of life, in any form.
 * 
 * Author : v.c
*/

#include <iostream>
#include <fstream>

#include "kf.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::vector<double> gt;
std::vector<double> measurement;
std::vector<double> corrected;

const double delta_t = 0.03; //time stamp for one sampling
const double initial_velocity = 1.33;
const double acc_var = 0.01;
const double meas_var = 0.1;

void process_data(std::ifstream& datafile)
{
    double t_meas=0.0, t_gt=0.0;
    while(datafile >> t_meas >> t_gt)
    {
        measurement.push_back(t_meas);
        gt.push_back(t_gt);
    }
}

void visualize()
{
    plt::figure_size(1200,780);
    plt::subplot(2,1,1);
    plt::named_plot("Ground Truth", gt, "b");
    plt::named_plot("Noisy Measurement", measurement, "r");
    plt::xlabel("Time");
    plt::ylabel("Distance (m)");
    plt::legend();

    plt::subplot(2,1,2);
    plt::named_plot("Ground Truth", gt, "b");
    plt::named_plot("Corrected output", corrected, "r");
    plt::xlabel("Time");
    plt::ylabel("Distance (m)");
    plt::legend();

    plt::show();
}

int main()
{
    std::ifstream dummy_data_file;
    dummy_data_file.open("../dummy_data.txt");
    if(!dummy_data_file.is_open())
    {
        std::cerr << "Error reading data file." << std::endl;
        exit(1);
    }
    process_data(dummy_data_file);

    //KalmanFilter(initial position, initial velocity, acceleration variance)
    KalmanFilter kf = KalmanFilter(measurement[0], initial_velocity, acc_var); 

    int n_meas = measurement.size();
    for(int i=1; i < n_meas; i++)
    {
        kf.predict(delta_t);
        kf.update(measurement[i], meas_var);
        corrected.push_back(kf.pos());
    }

    visualize();

    return 0;
}