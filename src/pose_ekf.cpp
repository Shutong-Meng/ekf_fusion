#include "pose_ekf.h"
#include <iostream>
#include <Eigen/Dense>
#include "conversion.h"
using namespace std;
using namespace Eigen;


//quaternion: body fram to navigation frame
//Rnb
// state for kalman filter
// 0-3 quaternion
// 4-6 Px Py Pz
// 7-9 Vx Vy Vz
// 10-12 bwx bwy bwz
// 13-15 bax bay baz 
// inertial frame: ENU

Matrix3d skew_symmetric(Vector3d v)
{
	Matrix3d m;
	m << 0, -v(2), v(1),
		 v(2), 0,  -v(0),
		 -v(1), v(0), 0;
	return m;
}

//diff_(p*q) /diff_q
Matrix4d diff_pq_q(Quaterniond p)
{
	double p0 = p.w();
	Vector3d pv = p.vec();

	Matrix4d D;
	D(0, 0) = p0;
	D.block<1, 3>(0, 1) = -pv.transpose();
	D.block<3, 1>(1, 0) = pv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*p0 + skew_symmetric(pv);
	return D;
}


//diff_(p*q)/ diff_p
Matrix4d diff_pq_p(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix4d D;
	D(0, 0) = q0;
	D.block<1, 3>(0, 1) = -qv.transpose();
	D.block<3, 1>(1, 0) = qv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*q0 - skew_symmetric(qv);
	return D;
}

//diff_(q*v*q_star)/ diff_q
MatrixXd diff_qvqstar_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v + skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() - q0*skew_symmetric(v));
	return D; 
}

//diff_(qstar*v*q)/ diff_q
MatrixXd diff_qstarvq_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v - skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() + q0*skew_symmetric(v));
	return D; 
}
//diff_(q*v*q_star)/ diff_v
Matrix3d diff_qvqstar_v(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix3d D;
	D = (q0*q0 - qv.dot(qv))*Matrix3d::Identity() + 2*qv*qv.transpose() + 2*q0*skew_symmetric(qv);
	return D; 
}

Pose_ekf::Pose_ekf()
{	
	
	x = VectorXd::Zero(n_state);  //initialize state matrix zero
	x.head(4) << 1, 0, 0, 0;
	x.segment<3>(10)=biw;
	x.segment<3>(13)=bia;
	P = MatrixXd::Identity(n_state, n_state);//dan wei juzhen   xiefangcha P

	Q = MatrixXd::Zero(6, 6);  //imu guance zaosheng
	Q.block<3, 3>(0, 0) = Matrix3d::Identity()*gyro_cov;
	Q.block<3, 3>(3, 3) = Matrix3d::Identity()*acc_cov;


	initialized = false;

	//imu_initialized = false;
}

Pose_ekf::~Pose_ekf()
{
	
}


void Pose_ekf::predict(Vector3d gyros, Vector3d acce, double t)
{	
	if(initialized==false)
	{
		//imu_initialized = true; 
		initialized = true;
		this->current_t = t;
		double phy = atan2(acce(1), acce(2));   //tan [-pi,pi]
		double theta = atan2(-acce(0), acce(2));
		Vector3d rpy(phy, theta, 0);
		Quaterniond q = euler2quaternion(rpy);
		x(0) = q.w(); 
		x.segment<3>(1) = q.vec();
		return;
	}

	if(t <= current_t) 
		return;

	double dt = t - current_t;//imu message hz
	//VectorXd xdot(n_state);//16*1 predicted state
	MatrixXd F(n_state, n_state);//16*16
	MatrixXd G(n_state, 6);//G = dx/du
 
	process(gyros, acce, x, F, G, dt);
	//x += xdot*dt;
	x.head(4).normalize();
	cout<<"Predict is \n"<<x<<endl;
	//F = MatrixXd::Identity(n_state, n_state) + F*dt;//continous F and discrete F  (f)
	// F= F*dt; ???
	// G = G*dt;  //(h)
	//cout << "F: " << F*P*F.transpose() << endl;
	//out << "G: " << G*Q*G.transpose() << endl;

	P = F*P*F.transpose() + G*Q*G.transpose();
	
	this->current_t = t;
	this->acc = acce;
	this->gyro = gyros;
	
}

//xdot = f(x, u);
void Pose_ekf::process(Vector3d gyros, Vector3d acce, VectorXd& xt, MatrixXd& F, MatrixXd& G, double t)
{
	
	Quaterniond q;
	Vector3d p, v, bw, ba;
	getState(q, p, v, bw, ba);
	
	F.setZero();
	G.setZero();
	
	Quaterniond gyro_q(0, 0, 0, 0);
	gyro_q.vec() = gyros - bw;
	Quaterniond q_dot = q*gyro_q;
	q_dot.w() *= t/2; q_dot.vec() *= t/2;//* and / to scalar is not support for Quaternion
	// xdot(0) = q_dot.w(); xdot.segment<3>(1) = q_dot.vec();
    // Vector3d gyro_v= gyros - bw;
    // Quaterniond q_dot = q * Quaterniond(1, gyro_v(0) * t / 2, gyro_v(1) * t / 2, gyro_v(2) * t / 2);
    xt(0) += q_dot.w(); 
    xt.segment<3>(1) += q_dot.vec();

	Quaterniond acc_b_q(0, 0, 0, 0);
	acc_b_q.vec() = acce - ba;
	Quaterniond acc_n_q =  q*acc_b_q*q.inverse();
	xt.segment<3>(7) += (acc_n_q.vec() - GRAVITY)*t;
	xt.segment<3>(4) += (acc_n_q.vec() - GRAVITY)*t*t/2;
	// xdot.segment<3>(7) = acc_n_q.vec() - GRAVITY;//body frame to n frame 
    // Vector3d un_acc = acce-ba;
    // cout<<"un_acc is \n"<<un_acc<<endl;
    // xt.segment<3>(4) = p + v * t + 0.5 * un_acc * t * t;
    // xt.segment<3>(7) = v + un_acc * t;
	//xdot.segment<3>(4) = v + xdot.segment<3>(7)*t;

	F.block<4, 4>(0, 0) = 0.5*diff_pq_p(gyro_q);
	F.block<4, 3>(0, 10) = -0.5*(diff_pq_q(q).block<4, 3>(0, 1));
	F.block<3, 3>(4, 7) = Matrix3d::Identity();
	F.block<3, 4>(7, 0) = diff_qvqstar_q(q, acc_b_q.vec());
	F.block<3, 3>(7, 13) = -diff_qvqstar_v(q);

	//G = d_xdot/du
	G.block<4, 3>(0, 0) = 0.5*diff_pq_q(q).block<4, 3>(0, 1);//diff(0.5*q*gyro_q)/diff(gyro_q)
	G.block<3, 3>(7, 3) = diff_qvqstar_v(q);//diff(q*a*qstar)/diff(a)
}

void Pose_ekf::getState(Quaterniond& q, Vector3d& p, Vector3d& v, Vector3d & bw, Vector3d& ba)
{
	q.w() = x(0);
	q.vec() = x.segment<3>(1);
	p = x.segment<3>(4);
	v = x.segment<3>(7);
	bw = x.segment<3>(10);
	ba = x.segment<3>(13);
}




void Pose_ekf::correct(VectorXd z, VectorXd presult, MatrixXd H, MatrixXd R)
{
   	MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    x += K*(z - presult);
    //cout<<"P"<<"\n"<<P<<endl;
    cout<<"correct is"<<"\n"<<K*(z-presult)<<endl;
    //cout<<"result is"<<"\n"<<x<<endl;
    MatrixXd I = MatrixXd::Identity(n_state, n_state);
    P = (I - K*H)*P;
    x.head(4).normalize();	
}

void Pose_ekf::measurement_slam(VectorXd& pose, MatrixXd &H)
{
	pose = x.segment<7>(0);
	H = MatrixXd::Zero(7, n_state); //H is used to calculate
	H.block<7, 7>(0, 0) = MatrixXd::Identity(7,7);
}
void Pose_ekf::correct_slam(Vector3d pos, Quaterniond q, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	if(t < current_t) return;
	predict(this->gyro, this->acc, t); //????
	
	VectorXd p(7);
	p(0)= q.w();
	p(1)= q.x();
	p(2)= q.y();
	p(3)= q.z();
	p(4)= pos(0);
	p(5)= pos(1);
	p(6)= pos(2);
	VectorXd presult;
	MatrixXd H;
	measurement_slam(presult, H); //modify
	correct(p, presult, H, R_fix);
}

void Pose_ekf::measurement_gravity(Vector3d& acc, MatrixXd& H)
{
	Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Vector3d ba = x.segment<3>(13);
	Quaterniond g_n_q;
	g_n_q.w() = 0; g_n_q.vec() = Vector3d(0, 0, 1);//only direction is used
	Quaterniond acc_q =  q.inverse()*g_n_q*q; //r_n to r_b
	acc = acc_q.vec();

	H = MatrixXd::Zero(3, n_state);
	H.block<3, 4>(0, 0) = diff_qstarvq_q(q, GRAVITY);
}

void Pose_ekf::correct_gravity(Vector3d acc)// double t
{
	// if(!initialized)
	// {
	// 	initialized = true;
	// 	this->current_t = t;
	// 	return;
	// }
	// if(t < current_t) return;
	// predict(this->gyro, this->acc, t);
	
	Vector3d z = acc/acc.norm();
	Vector3d presult;
	MatrixXd H;
	measurement_gravity(presult, H);
	correct(z, presult, H, R_gravity);
}