#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
	is_initialized_ = false;
	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
				0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;

	/** TODO:
	* Finish initializing the FusionEKF.
	* Set the process and measurement noises
	*/

	// measurement matrix for laser just needs position values - ignore velocity
	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;

	// An H matrix for Radar is calculated on the fly each time we get a radar measurement

	noise_ax = 20;
	noise_ay = 20;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::UpdateStateTransMatrixF(float dt) {
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
}

void FusionEKF::UpdateProcessCovarianceMatrixQ(float dt) {
    float dt2 = pow(dt, 2);
    float dt3 = pow(dt, 3);
    float dt4 = pow(dt, 4);

    ekf_.Q_(0, 0) = dt4/4 * noise_ax;
    ekf_.Q_(0, 2) = dt3/2 * noise_ax;
    ekf_.Q_(1, 1) = dt4/4 * noise_ay;
    ekf_.Q_(1, 3) = dt3/2 * noise_ay;
    ekf_.Q_(2, 0) = dt3/2 * noise_ax;
    ekf_.Q_(2, 2) = dt2 * noise_ax;
    ekf_.Q_(3, 1) = dt3/2 * noise_ay;
    ekf_.Q_(3, 3) = dt2 * noise_ay;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized_) {
		/** TODO:
		* Initialize the state ekf_.x_ with the first measurement.
		* Create the covariance matrix.
		* Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/

		// first measurement
		cout << "EKF: " << endl;

		// KF init takes: xPFHRQ

		VectorXd x_init = VectorXd(4);
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			// Convert radar from polar to cartesian coordinates
			cout << "Received a RADAR measurement first" << endl;
			float r = measurement_pack.raw_measurements_[0];
			float radians = measurement_pack.raw_measurements_[1];
			x_init << r*cos(radians), r*sin(radians), 0, 0;
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			// Initialize state with location and zero velocity
			cout << "Received a LIDAR measurement first" << endl;
			x_init << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		}

		MatrixXd P_init = MatrixXd(4, 4);
		P_init << 10, 0, 0, 0,
				  0, 10, 0, 0,
				  0, 0, 1000, 0,
				  0, 0, 0, 1000;

		MatrixXd F_init = MatrixXd(4, 4);
		F_init << 1, 0, 1, 0,
				  0, 1, 0, 1,
				  0, 0, 1, 0,
				  0, 0, 0, 1;

		MatrixXd Q_init = MatrixXd(4, 4);
		Q_init << 0, 0, 0, 0,
				  0, 0, 0, 0,
				  0, 0, 0, 0,
				  0, 0, 0, 0;

		// H and R get set before the update is called based on input type, so need to differentiate here
		ekf_.Init(x_init, P_init, F_init, H_laser_, R_laser_, Q_init);
		cout << "Initialized EKF" << endl;

		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;

		return;  // done initializing, no need to predict or update
	}

	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	/*****************************************************************************
	*  Prediction
	****************************************************************************/

	/** TODO:
	* Update the state transition matrix F according to the new elapsed time.
		- Time is measured in seconds.
	* Update the process noise covariance matrix.
	* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/

	string m_type;
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) m_type = "RADAR";
	else m_type = "LIDAR";

	// cout << "==== RECEIVED " << m_type << " ====" << endl;
	// cout << "==== Here's the RAW " << m_type << ": ====" << endl;
	// cout << measurement_pack.raw_measurements_ << endl;
	// cout << "==== END RAW ====" << endl;

	UpdateStateTransMatrixF(dt);
	UpdateProcessCovarianceMatrixQ(dt);

	// cout << "Predicting..." << endl;
	ekf_.Predict();

	/*****************************************************************************
	*  Update
	****************************************************************************/

	/** TODO:
	* Use the sensor type to perform the update step.
	* Update the state and covariance matrices.
	*/

	VectorXd z = measurement_pack.raw_measurements_;
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		ekf_.R_ = R_radar_;
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		// cout << "Updating for RADAR..." << endl;
		ekf_.UpdateEKF(z);
	}
	else {
		// Laser updates
		ekf_.R_ = R_laser_;
		ekf_.H_ = H_laser_;
		// cout << "Updating for LIDAR..." << endl;
		ekf_.Update(z);
	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
