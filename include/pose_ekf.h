#ifndef __POSE_EKF_H
#define __POSE_EKF_H
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// state for kalman filter
// 0-3 quaternion
// 4-6 Px Py Pz
// 7-9 Vx Vy Vz
// 10-12 bwx bwy bwz
// 13-15 bax bay baz 
// inertial frame: ENU

class Pose_ekf

{
public:
	Pose_ekf();
	~Pose_ekf();	
	void predict(Vector3d , Vector3d , double );
	//void correct(Vector3d pos, Vector3d vel, Vector3d mag, double t);
	//void process(Vector3d gyros, Vector3d acce, VectorXd& xdot, MatrixXd& F, MatrixXd& G, double t);
	void process(Vector3d , Vector3d , VectorXd& , MatrixXd& , MatrixXd& , double );
	//MatrixXd computeF(Vector3d gyro, Vector3d acc);
	
	//VectorXd measurement(VectorXd x, Vector3d mag);
	//MatrixXd computeH(Vector3d mag);

	// void measurement_fix(Vector2d& position, MatrixXd &H);
	// void measurement_fix_velocity(Vector3d& velocity, MatrixXd& H);
	// void measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H);
	// void measurement_magnetic_field(Vector3d& magnetic_field, MatrixXd& H);
	void measurement_gravity(Vector3d& acc, MatrixXd& H);
	void measurement_slam(VectorXd& pose, MatrixXd &H);

	//zidingyi
	void correct_slam(Vector3d pos, Quaterniond q, double t);
	void correct(VectorXd z, VectorXd presult, MatrixXd H, MatrixXd R);


	
	// void correct_fix(Vector3d position, double t);
	// void correct_fix_velocity(Vector3d velocity, double t);
	// void correct_sonar_height(double sonar_height, double t);//todo, without considering the roll and pitch
	// void correct_magnetic_field(Vector3d mag, double t);
	void correct_gravity(Vector3d acc, double t);
	// void measurement_altimeter(double& altimeter_height, MatrixXd H);
	void getState(Quaterniond& q, Vector3d& position, Vector3d& velocity, Vector3d & bw, Vector3d&  ba);
	double get_time() { return current_t;}
private:
	VectorXd x;//state 
	MatrixXd P;//covariance
	MatrixXd Q;//imu observation noise
	const MatrixXd R_fix = MatrixXd::Identity(7,7)*fix_cov;;
	const Vector3d GRAVITY = Vector3d(0, 0, 9.8);
	//covariance parameter
	const double fix_cov = 0.1;//smaller and more reliable to measurement.
	
	const double gyro_cov =1.6968e-04;
	const double acc_cov = 2.0e-3;
	const Vector3d bia = Vector3d(3.0000e-3,3.0000e-3,3.0000e-3);
	const Vector3d biw = Vector3d(1.9393e-05,1.9393e-05,1.9393e-05);
	const double gravity_cov = 5.0;

	const int n_state = 16;

	//const MatrixXd R_fix = Matrix2d::Identity()*fix_cov;
	
	const MatrixXd R_gravity = Matrix3d::Identity()*gravity_cov;

	Vector3d acc;
	Vector3d gyro;

	double current_t;

	bool initialized = false;
	//bool imu_initialized;
	
};

#endif 
