#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		MatrixXd Jv = MatrixXd::Zero(3,dof);
		MatrixXd Lambda = MatrixXd::Zero(3,3);
		Matrix3d R;
		MatrixXd J_bar = MatrixXd::Zero(dof,3);
		MatrixXd N = MatrixXd::Zero(dof,dof);
		VectorXd g = VectorXd::Zero(dof);
		VectorXd b = VectorXd::Zero(dof);
		Vector3d x;
		Vector3d v;
		VectorXd F(dof);
		double kp = 100;
		double kv = 20;
		double kpj = 50;
		double kvj = 14;
		robot->Jv(Jv, link_name, pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->gravityVector(g);
		robot->nullspaceMatrix(N,Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->position(x, link_name, pos_in_link);
		robot->linearVelocity(v, link_name, pos_in_link);
		robot->rotation(R,link_name);
		auto q = robot->_q;
		int controller_number = QUESTION_3;

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			Vector3d xd(0.3, 0.1, 0.5);
			Vector3d xdd(cos(M_PI*time),-sin(M_PI*time),0);
			xdd = xdd * 0.1 * M_PI;
			Vector3d xddd(-sin(M_PI*time),-cos(M_PI*time),0);
			xddd = xddd * 0.1 * M_PI * M_PI;
			xd(0) += 0.1 * sin(M_PI * time);
			xd(1) += 0.1 * cos(M_PI * time);
			// F = Lambda * (kp * (xd - x) - kv * v); //1a
			F = Lambda * (xddd + kp * (xd - x) - kv * (v-xdd)); //1b
			command_torques = Jv.transpose() * F - N.transpose() * (kvj * robot->_dq + kpj * q) + g;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			// Vector3d xd(-0.1, 0.15, 0.2);
			Vector3d xd(-0.65, -0.45, 0.7);
			VectorXd q_lo(dof);
			q_lo << -165.0,-100.0,-165.0,-170.0,-165.0,0.0,-165.0;
			VectorXd q_hi(dof);
			q_hi << 165.0,100.0,165.0,-30.0,165.0,210.0,165.0;
			q_lo *= M_PI / 180.0;
			q_hi *= M_PI / 180.0;
			VectorXd q_d = (q_hi + q_lo)/2.0;
			// cout<<q_d<<endl;
			VectorXd L_mid = 2*(-25.0)*(q-q_d);
			VectorXd L_damp = - 14.0 * robot->_dq;
			F = Lambda * (kp * (xd - x) - kv * v);
			// command_torques = Jv.transpose() * F + N.transpose() * L_damp + g; //2d
			// command_torques = Jv.transpose() * F + N.transpose() * (L_damp+L_mid) + g; //2ef
			command_torques = Jv.transpose() * F + N.transpose() * L_damp+ L_mid + g; //2g
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			Vector3d xd(0.6, 0.3, 0.5);
			Matrix3d Rd;
			MatrixXd J(6,dof);
			VectorXd help(6);
			Vector3d w;
			robot->J_0(J,link_name, pos_in_link);
			robot->angularVelocity(w,link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, J);
			Rd<<cos(M_PI/3.0),0,sin(M_PI/3.0),
					0,1,0,
					-sin(M_PI/3.0),0,cos(M_PI/3.0);
			Vector3d delta_phi(0.0,0.0,0.0);
			for(int i=0; i < 3;i++){
				delta_phi += R.row(i).cross(Rd.row(i));
			}
			kp = 80;
			kv = 17;
			delta_phi *= -0.5;
			help << kp*(xd-x)-kv*v, kp*(-delta_phi)-kv*w;
			F = Lambda * help;
			command_torques = J.transpose() * F - N.transpose()*kvj*robot->_dq + g;
			redis_client.setEigenMatrixDerived("dphi",delta_phi);
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			Vector3d xd(0.6, 0.3, 0.4);
			kp = 200.0;
			kv = 50.0;
			double Vmax = 0.1;
			Vector3d vd = kp/kv * (xd-x);
			double nu = Vmax / vd.norm();
			// clamp(v,-1,1);

			// F = Lambda * (kp*(xd-x)-kv*v); //4a
			F = Lambda * (-kv * (v-nu*vd)); //4b
			command_torques = Jv.transpose() * F - N.transpose() * robot->_M * (kvj * robot->_dq + kpj * q) + g;
		}
		redis_client.setEigenMatrixDerived("x",x);
		redis_client.setEigenMatrixDerived("v",v);

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
