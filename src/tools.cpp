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
	if(estimations.size() != ground_truth.size())
		std::cout << "sizes dont match for rmse calculations" << std::endl;

	for(int i = 0; i < estimations.size(); i++){
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = pow(residual.array(), 2);
		rmse += residual;
	}

	rmse /= estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	if(fabs(px) < 0.0001){
	px = 0.0001;
	py = 0.0001;
	}

	double c1 = pow(px, 2) + pow(py, 2);
	if(fabs(c1) < 0.000001)
	c1 = 0.000001;

	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	MatrixXd cov(3,4);
	cov << (px/c2), (py/c2), 0, 0,
		   -(py/c1), (px/c1), 0, 0,
		   py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
	return cov;
   
}
