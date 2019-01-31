#pragma once
#include "maxonMotor.h"
#include <Eigen/Dense>
#include "kinematics.h"
#include <chrono>
#include <thread>
#include <functional>
#include <iostream>
#include <fstream>
using namespace Eigen;
using namespace std;

const static IOFormat CSVtmp2(StreamPrecision, DontAlignCols, ", ", "\n");

//template<typename M>
//M load_csv(const std::string & path) {
//	std::ifstream indata;
//	indata.open(path);
//	std::string line;
//	std::vector<double> values;
//	uint rows = 0;
//	while (std::getline(indata, line)) {
//		std::stringstream lineStream(line);
//		std::string cell;
//		while (std::getline(lineStream, cell, ',')) {
//			values.push_back(std::stod(cell));
//		}
//		++rows;
//	}
//	return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size() / rows);
//}

void convertEncReadings(long* posInts, double* posAct, long* initOffset) {
	posAct[0] = ((double)posInts[0] + initOffset[0]) * M_PI / 2048;
	posAct[1] = ((double)posInts[1] + initOffset[1]) * M_PI / 2048;
	posAct[2] = ((double)posInts[2] + initOffset[2]) * 25.4/2000;
	posAct[3] = ((double)posInts[3] + initOffset[3]) * 12 / 2000;
}

void setStartPosition(long *CurrentPositions, long *initOffset, maxonMotor motors) {
	//Initialise Start Position
	motors.DisableAll();
	cout << "Move Robot to initialisation position, press enter when done" << endl;
	getchar();
	motors.GetMotorPositions(CurrentPositions);
	initOffset[0] = -2048 - CurrentPositions[0]; //Yaw start
	initOffset[1] = -521 - CurrentPositions[1]; //Pitch start
	initOffset[2] = 6705.5 - CurrentPositions[2]; //Linear start
	initOffset[3] = 4122 - CurrentPositions[3]; //Transverse start
	cout << "Initialisation complete" << endl << endl;
	//for (int i = 0; i < 4; i++) {cout << CurrentPositions[i] << " ";}
	//cout << endl;
	//for (int i = 0; i < 4; i++) {cout << initOffset[i] << " ";}
	//cout << endl;
}


void Calibration(maxonMotor motors, int numReadings, int timePeriod, long* initOffset) {
	cout << "Press enter to start calibration" << endl;
	getchar();

	int numberRead = 0;
	long intPos[4] = { 0,0,0,0 };
	double currPos[4] = { 0, 0, 0 , 0};
	LARGE_INTEGER startTime, prevTime, elapsedTime, freq;
	MatrixXd encReadings(5, numReadings); //Array for all encoder readings
	MatrixXd endPos(3, numReadings);
	QueryPerformanceCounter(&prevTime);
	while (numberRead < numReadings)
	{
		auto x = chrono::steady_clock::now() + chrono::milliseconds(timePeriod);
		QueryPerformanceCounter(&startTime);
		QueryPerformanceFrequency(&freq);
		motors.GetMotorPositions(intPos);
		convertEncReadings(intPos, currPos, initOffset);
		elapsedTime.QuadPart = startTime.QuadPart - prevTime.QuadPart;
		elapsedTime.QuadPart *= 1000000;
		elapsedTime.QuadPart /= freq.QuadPart;
		double tmp2[] = { currPos[0], currPos[1], currPos[2], currPos[3], (double) elapsedTime.QuadPart };
		//cout << currPos[0] << " and " << currPos[1] <<  " and " << elapsedTime.QuadPart << endl;
		Map<Matrix<double, 5, 1>> readings(tmp2);
		encReadings.col(numberRead) = readings;
		numberRead++;
		prevTime.QuadPart = startTime.QuadPart;
		while (chrono::steady_clock::now() < x) {};
	}
	//long posAct[4];
	//for (int i = 0; i < numReadings; i++){
	//	posAct[4] = 
	//	encReadings.col(i)

	//}
	cout << "Calibration Finished" << endl;
	string name = "tomTestFINAL.csv";
	ofstream myFile(name.c_str());
	myFile << encReadings.format(CSVtmp2);
	getchar();
}

Vector3d getEndPosition(maxonMotor motors, long* initOffset, Matrix4d DH) {
	Matrix4d T;
	Vector3d endPos;
	//DH cols: d, t, r, alpha
	long CurrentPositions[4];
	double actPositions[4];
	motors.GetMotorPositions(CurrentPositions);
	convertEncReadings(CurrentPositions, actPositions, initOffset);
	T = forwardKinematics3(DH, actPositions);
	endPos = T.block<3, 1>(0, 3);
	return endPos;
}

Matrix3d planePoints(maxonMotor motors, long* initOffset, Matrix4d DH, double* point1) {
	cout << "Define plane" << endl; 
	Matrix3d planeP;
	long currInt[4] = { 0,0,0,0 };
	for (int i = 0; i < 3; i++) {
		cout << "Move to position " << i << " and press enter" << endl;
		getchar();
		if (i == 0) {
			motors.GetMotorPositions(currInt);
			convertEncReadings(currInt, point1, initOffset);
			//for (int j = 0; j < 4; j++) { cout << point1[j] << " "; } 
			//cout << endl;
		}
		planeP.col(i) = getEndPosition(motors, initOffset, DH);
	}
	return planeP;
}

void followTrajectory(maxonMotor motors, MatrixXi velJoints, double dt, long* initOffset, Matrix4d DH) {
	int numIter = velJoints.cols();
	double tmp = dt * 1000;
	int timePeriod = (int)tmp;
	int tmpVels[4] = { 0,0,0,0 };
	int targetVels[4] = { 0,0,0,0 };
	long currInt[4] = { 0,0,0,0 };
	double posAct[4];
	MatrixXd currArray(4, numIter);

	cout << "Press Enter to start writing!" << endl;
	motors.SetAllVelocityMode();
	motors.EnableAll();
	motors.SetSpeeds(targetVels);
	getchar();
	for (int i = 0; i < numIter; i++) {
		auto x = chrono::steady_clock::now() + chrono::milliseconds(timePeriod);
		motors.GetMotorPositions(currInt);
		convertEncReadings(currInt, posAct, initOffset);
		Map<Vector4d> currPosVec(posAct);
		currArray.col(i) = currPosVec;

		Vector4i::Map(tmpVels) = velJoints.col(i);
		targetVels[0] = (long)tmpVels[3];
		targetVels[1] = (long)tmpVels[2];
		targetVels[2] = (long)tmpVels[0];
		targetVels[3] = (long)tmpVels[1];
		motors.SetSpeeds(targetVels);
		while (chrono::steady_clock::now() < x);
	}
	cout << "Finished, press Enter to close" << endl;
	motors.Home(initOffset);
	MatrixXd endPos(3, numIter);

	for (int i = 0; i < numIter; i++) {
		Vector4d::Map(posAct) = currArray.col(i);
		Matrix4d T;
		T = forwardKinematics3(DH, posAct);
		endPos.col(i) = T.block<3, 1>(0, 3);
	}
	writeToCSVfile("currArray.csv", currArray);
	writeToCSVfile("linePosAct.csv", endPos);

	getchar();
}

void updateDH(maxonMotor motors, Matrix4d DH, long* initOffset) {
	long CurrentPositions[4] = { 0,0,0,0 };
	motors.GetMotorPositions(CurrentPositions);
	double jointAct[4] = { 0,0,0,0 };
	convertEncReadings(CurrentPositions, jointAct, initOffset);
	DH(1, 0) = jointAct[2];
	DH(2, 0) = jointAct[3];
	DH(2, 1) = jointAct[1];
	DH(3, 1) = jointAct[0];
	cout << DH << endl;
}

