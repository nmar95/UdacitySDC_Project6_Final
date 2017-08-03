#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*Constructor.*/
FusionEKF::FusionEKF() {
    // Initializing matrices (Starter Code)
    is_initialized_ = false;
    previous_timestamp_ = 0;

    R_laser_ = MatrixXd(2, 2); //Measurement covariance matrix for Laser
    R_radar_ = MatrixXd(3, 3); //Measurement covariance matrix for Radar
    H_laser_ = MatrixXd(2, 4); //Measurement matrix for Laser
    Hj_ = MatrixXd(3, 4); //Measurement matrix for Radar (Jacobian Matrix)

    // Measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    // Measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    /* TODO:
      1. Finish initializing the FusionEKF.
      2. Set the process and measurement noises
    */

      // MY CODE

      H_laser_ << 1,0,0,0,
                  0,1,0,0;

      Hj_ << 0,0,0,0,
             0,0,0,0,
             0,0,0,0;

      // Previously coded this, but commented it out
      //Probably dont need because you initialize it below
      //Create a 4D state vector, we don't know yet the values of the x state
      //ekf_.x_ = VectorXd(4);

      //State covariance matrix P
      ekf_.P_ = MatrixXd(4, 4);
      ekf_.P_ << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1000, 0,
                 0, 0, 0, 1000;

      //The initial transition matrix F_
      ekf_.F_ = MatrixXd(4, 4);
      ekf_.F_ << 1, 0, 1, 0,
                 0, 1, 0, 1,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
}

/*Destructor*/
FusionEKF::~FusionEKF() {}

/* Create Process Measurement Function */
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization - Completed
   ****************************************************************************/

   if (!is_initialized_) {
      /* TODO:
       1. Initialize the state ekf_.x_ with the first measurement.
       2. Create the covariance matrix.
       */

      // MY CODE:

      // First measurement - 4D state vector
      cout << "EKF: " << endl;
      ekf_.x_ = VectorXd(4);
      ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /*
             1. Convert radar from polar to cartesian coordinates
             2. Initialize state.
             */
            // MY CODE:
            float rho = measurement_pack.raw_measurements_[0];  //Range
            float phi = measurement_pack.raw_measurements_[1];  // Bearing

            //Change from Polar to Cartesian
            float px_cart = rho*cos(phi);
            float py_cart = rho*sin(phi);

            //Initialize State variables px and py
            ekf_.x_ << px_cart, py_cart, 0, 0;

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            //1. Initialize state.
            // MY CODE:
            //Initialize state - (Quiz 2)
            ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1], 0, 0;
        }

        // Update Previous Timestamp
        previous_timestamp_ = measurement_pack.timestamp_;

        // Done initializing, no need to predict or update
        is_initialized_ = true;
        return;
      }

  /*****************************************************************************
   *  Prediction - Completed
   ****************************************************************************/
   /*TODO:
    1. Update the state transition matrix F according to the new elapsed time (seconds)
    2. Update the process noise covariance matrix. (noise_ax=9 & noise_ay=9 for Q matrix)
    */

    // MY CODE

    // Set the acceleration noise components
    float noise_ax = 9;
    float noise_ay = 9;

    // Compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // Set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    // Call the Predict Function now that F and Q are have been updated
    ekf_.Predict();

  /*****************************************************************************
   *  Update - Completed
   ****************************************************************************/

   /*TODO:
    1. Use the sensor type to perform the update step.
    2. Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates. Update the state and covariance matrices.
        // MY CODE

        // Initialize the Jacobian Matrix (Hj_) in the tools.cpp file
        Hj_ = tools.CalculateJacobian(ekf_.x_);

        // Set the H_ matrix equal to the Hj_ matrix
        ekf_.H_ = Hj_;

        // Set covariance matrix equal to the Radar measurement covariance matrix(R)
        ekf_.R_ = R_radar_;

        // Call the UpdateEKF() function
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates. Update the state and covariance matrices.
        // MY CODE

        // Set the H Matrix
        ekf_.H_ = H_laser_;

        // Set the R Matrix
        ekf_.R_ = R_laser_;

        // Call the Update() function
        ekf_.Update(measurement_pack.raw_measurements_);
    }

  // Print the output (x vector and covariance matrix)
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
