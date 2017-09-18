#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(
	VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	/** TODO: predict the state */
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;

	cout << "Prediction:" << endl;
	cout << "x_ = " << x_ << endl;
	// cout << "P_ = " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
	/** TODO: update the state by using Kalman Filter equations */
	VectorXd y = z - H_ * x_;
	cout << "y: " << y << endl;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd K = P_ * Ht * S.inverse();

	//new estimate
	x_ = x_ + K * y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/** TODO: update the state by using Extended Kalman Filter equations
	* H_ has been set to Hj
	*/
	MatrixXd Hj = H_;
	// cout << "Calculating y" << endl;
	// cout << "Polar x: " << tools.ConvertToPolar(x_) << endl;
	VectorXd y = z - tools.ConvertToPolar(x_);
	// cout << "y before normalize: " << y << endl;
	tools.NormalizeAngle(y);
	cout << "y: " << y << endl;
	// cout << "Calculating Hjt" << endl;
	MatrixXd Hjt = Hj.transpose();
	// cout << "Calculating S" << endl;
	MatrixXd S = Hj * P_ * Hjt + R_;
	// cout << "Calculating K" << endl;
	MatrixXd K = P_ * Hjt * S.inverse();

	//new estimate
	// cout << "Calculating x" << endl;
	x_ = x_ + K * y;
	long x_size = x_.size();
	// cout << "Calculating I" << endl;
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	// cout << "Calculating P" << endl;
	P_ = (I - K * H_) * P_;
}
