#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
    
    //measurement covariance
    ekf_.R_ = R_laser_;
    
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    
    
    //create a 4D state vector, we don't know yet the values of the x state
    ekf_.x_ = VectorXd(4);
    
    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
    
    
    //measurement matrix
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ << 1, 0, 0, 0,
    0, 1, 0, 0;
    
    //the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
         TODO:
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */
        // first measurement
//        cout << "EKF test: " << endl;

        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            double rho_measured_ = measurement_pack.raw_measurements_[0];
            double phi_measured_ = measurement_pack.raw_measurements_[1];
            
//            cout << "rho_measures = " << rho_measured_ << endl;
//            cout << "phi_measured_ = " << phi_measured_ << endl;

            ekf_.x_ << cos(phi_measured_)*rho_measured_, sin(phi_measured_)*rho_measured_, 0, 0;
            previous_timestamp_ = measurement_pack.timestamp_;

            //rho = object range (radial distance from origin)
            //phi = bearing: angle between rho and x
            //roh dot = range rate = radial velocity = change of rho

            
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            //set the state with the initial location and zero velocity
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
            
            previous_timestamp_ = measurement_pack.timestamp_;
        }
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    /**
     TODO:
     * Update the state transition matrix F according to the new elapsed time.
     - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    
    float noise_ax=9;
    float noise_ay=9;
    
    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    
    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    
    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
    0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
    dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    /**
     TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
//        cout << "raw measurements radar: " <<measurement_pack.raw_measurements_ << endl;
        ekf_.R_ = R_radar_;

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
       
        // Radar updates
    } else {
        ekf_.R_ = R_laser_;

       ekf_.Update(measurement_pack.raw_measurements_);

        
        // Laser updates
    }
    
//    // print the output
//    cout << "x_ = " << ekf_.x_ << endl;
//    cout << "P_ = " << ekf_.P_ << endl;
//    cout << "F_ = " << ekf_.F_ << endl;
    
}
