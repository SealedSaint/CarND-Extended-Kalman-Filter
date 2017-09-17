#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
	KalmanFilter();
	virtual ~KalmanFilter();

	VectorXd x_;  // state vector
	MatrixXd P_;  // state covariance matrix
	MatrixXd F_;  // state transition matrix
	MatrixXd Q_;  // process covariance matrix
	MatrixXd H_;  // measurement matrix
	MatrixXd R_;  // measurement covariance matrix

	/** Initializes the Kalman filter
	* @param x_in Initial state
	* @param P_in Initial state covariance
	* @param F_in Transition matrix
	* @param H_in Measurement matrix
	* @param R_in Measurement covariance matrix
	* @param Q_in Process covariance matrix
	*/
	void Init(
		VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
		MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in
	);

	/** Predicts the state and the state covariance using the process model */
	void Predict();

	/** Updates the state by using standard Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void Update(const VectorXd &z);

	/** Updates the state by using Extended Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void UpdateEKF(const VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
