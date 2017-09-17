#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "measurement_package.h"
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionEKF {
public:
	FusionEKF();
	virtual ~FusionEKF();

	void ProcessMeasurement(const MeasurementPackage &measurement_pack);  // Runs the flow of the Kalman Filter

	KalmanFilter ekf_;  // Kalman Filter update and prediction math lives here

private:
	void UpdateStateTransMatrixF(float dt);
	void UpdateProcessCovarianceMatrixQ(float dt);

	bool is_initialized_;  // check whether the tracking toolbox was initialized or not (first measurement)
	long long previous_timestamp_;
	float noise_ax;
	float noise_ay;

	Tools tools;  // Used to compute Jacobian and RMSE

	MatrixXd R_laser_;
	MatrixXd R_radar_;
	MatrixXd H_laser_;
	MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
