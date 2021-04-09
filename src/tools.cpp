#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	if(estimations.size() != ground_truth.size() || estimations.size() == 0){
		std::cout << "sizes dont match for rmse calculations" << std::endl;
		return rmse;
	}
	for(int i = 0; i < estimations.size(); i++){
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	rmse /= estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd cov(3,4);
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	float c1 = pow(px, 2) + pow(py, 2);
	float c2 = sqrt(c1);
	float c3 = (c1*c2);
	
	if(fabs(c1) < 0.0001){
		std::cout << "division by 0 in Jacobian calculation" << std::endl;
		return cov;
	}
		
	cov << (px/c2), (py/c2), 0, 0,
		   -(py/c1), (px/c1), 0, 0,
		   py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
	return cov;
   
}
