#include <iostream>
#include <math.h>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	/** TODO: Calculate the RMSE here. */

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.empty()){
		cout << "No estimations provided. Unable to calculate RMSE." << endl;
		return rmse;
	}
	if(estimations.size() != ground_truth.size()){
		cout << "Estimations and ground truth samples not equal in size. Unable to calculate RMSE." << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		// ... your code here
		VectorXd residual = estimations[i] - ground_truth[i];
		VectorXd residual_squared = residual.array() * residual.array();
		rmse += residual_squared;
	}
	//calculate the mean
	rmse /= estimations.size();
	//calculate the squared root
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/** TODO: Calculate a Jacobian here. */

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if(px == 0 && py == 0) {
		cout << "Division by zero would occur! Returning initialized Hj." << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	float squared_sum = pow(px, 2) + pow(py, 2);

	float v00 = px / sqrt(squared_sum);
	float v01 = py / sqrt(squared_sum);
	float v10 = -py / squared_sum;
	float v11 = px / squared_sum;
	float v20 = py*(vx*py - vy*px) / pow(squared_sum, 3/2);
	float v21 = px*(vy*px - vx*py) / pow(squared_sum, 3/2);
	float v22 = v00;
	float v23 = v01;

	Hj << v00, v01, 0, 0,
		  v10, v11, 0, 0,
		  v20, v21, v22, v23;

	return Hj;
}

VectorXd Tools::ConvertToPolar(const VectorXd& x_state) {
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	VectorXd polar = VectorXd(3);
	float c1 = sqrt(px*px + py*py);
	polar << c1,
			 atan2(py, px),  // atan2 considers the sign of both arguments to determine quadrant (returns [-pi, pi] instead of [-pi/2, pi/2])
			 (px*vx + py*vy) / c1;

	return polar;
}

void Tools::NormalizeAngle(VectorXd& polar) {
	float angle = polar(1);
	while(angle < -M_PI || angle > M_PI) {
		if(angle < -M_PI) {
			angle += 2*M_PI;
		}
		else {
			angle -= 2*M_PI;
		}
	}
	polar(1) = angle;
}
