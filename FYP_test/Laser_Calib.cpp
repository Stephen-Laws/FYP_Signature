#include "stdafx.h"
#include "kinematics.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "stringEditor.h"
#include "Definitions.h"
#include "maxonMotor.h"
#include <windows.h>
#include "testingFuncs.h"
using namespace Eigen;
using namespace std;

# define M_PI           3.14159265358979323846  /* pi */

int main()
{
	//Initialise Motors
	char* portName1 = const_cast<char*>("USB0");
	maxonMotor motors(portName1);
	long currPosInt[4] = { 0,0,0,0 };
	long initOffset[4] = { 0,0,0,0 };
	double point1[4] = { 0,0,0,0 };
	//Variable Initialisation
	char textToPrint[] = "ROBOT";//String to be traced

	//Kinematic Variables
	Matrix<double, Dynamic, 3> XYCoords, robotCoords;
	double dt = 0.015;
	double T = 0.1; //Time for movement /mm
	Vector3d endPos;
	Matrix3d planePos, planeRotMat;
	Matrix<double, 4, Dynamic> velJoints;
	Matrix<int, 4, Dynamic> intVels;
	double jointPos1[4] = { 0,0,0,0 };

	Matrix4d DH; //Cols: d, t, r, alpha
	//MatrixXi intVels;
	DH << 0, 0, 0, 0,
		0, 1.574, 51.45, -1.509,
		0, 0, -0.05063, -1.566,
		115.5, 0, 191.5, 0;


	motors.ActivateAll();
	motors.clearFaults();
	setStartPosition(currPosInt, initOffset, motors);
	//motors.Home(initOffset);
	planePos = planePoints(motors, initOffset, DH, jointPos1); //Define Plane
	planeRotMat = refAxes(planePos);
	cout << "Enter to move to start" << endl;
	getchar();
	motors.MoveToPos(jointPos1, initOffset);

	XYCoords = Coords(textToPrint); //text in XY plane
	XYCoords = scaleString(XYCoords, planePos.col(0), planePos.col(1));
	robotCoords = convertCoords(XYCoords, planeRotMat, planePos.col(0));//Text conversion to robot frame

	velJoints = getVelocities(robotCoords, DH, jointPos1, T, dt * 20);//Calculate Joint Velocities (rpm)
	intVels = decimaliseVelocities(velJoints, dt * 20, dt, jointPos1, DH);
	followTrajectory(motors, intVels, dt, initOffset, DH);//Move along calced velocities
	getchar();
	motors.DisableAll();
	motors.CloseAll();
}
