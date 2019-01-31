#pragma once
#include "stdafx.h"
//#include "stringEditor.h"
#include <iostream>
#include <Eigen/Dense>
#include<Eigen/Core>
#include<Eigen/SVD>
#include <fstream>
using namespace Eigen;
using namespace std;
const static IOFormat CSVtmp(StreamPrecision, DontAlignCols, ", ", "\n");

# define M_PI           3.14159265358979323846  /* pi */

void writeToCSVfile2(string name, MatrixXd matrix)
{
	ofstream myFile(name.c_str());
	myFile << matrix.format(CSVtmp);
}
//FUNCTIONS FOR PLANE DEFINITION
//Fucntion to defines axes from 3 points
//3 points are p1, top left, p2 top right, p3 bottom left
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * 4 * svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

void convertVelocities(MatrixXd velJoints) {
	velJoints.row(0) = velJoints.row(0) / 25.4 * 60;//d1
	velJoints.row(1) = velJoints.row(1) / 12 * 60;
	velJoints.row(2) = velJoints.row(2) / M_PI * 30;
	velJoints.row(3) = velJoints.row(3) / M_PI * 30;
}


Matrix3d refAxes(Matrix3d planePoints) {
	Vector3d p1,p2,p3,p12, p31,x,y,z, norm; 
	Matrix3d RotMat;
	p1 = planePoints.col(0);
	p2 = planePoints.col(1);
	p3 = planePoints.col(2);
	x = p2 - p1;
	x.normalize();
	p31 = p1 - p3;

	z = x.cross(p31);
	z.normalize();

	y = z.cross(x);
	y.normalize();
	RotMat << x, y, z; // Rotation Matrix of normalised x,y,z axes
	return RotMat;
}
MatrixXd convertCoords(MatrixXd points, MatrixXd rotMat, Vector3d translate) {
	int numRows = points.rows();
	int numCols = points.cols();
	//cout << translate << endl;
	VectorXd offPlane(numRows);
	offPlane.setZero();
	MatrixXd robotCoords(numRows, numCols);
	robotCoords.row(0) = translate; //Start at point 1
	for (int i = 1; i < numRows; i++) {
		robotCoords.row(i) = rotMat * points.row(i).transpose() + translate;
		if (points(i, 2) != 0) {
			offPlane(i) = 1;
		}
	}
	//cout << robotCoords << endl;
	return robotCoords;
}

//Kinematic Functions

MatrixXd jacobian(Matrix4d DH) {
	Vector4d d, theta, r, alpha, st, ct,sa,ca;
	d = DH.col(0);
	theta = DH.col(1);
	r = DH.col(2);
	alpha = DH.col(3);
	for (int i = 0; i < 4; i++) {
		st(i) = sin(theta(i));
		ct(i) = cos(theta(i));
		sa(i) = sin(alpha(i));
		ca(i) = cos(alpha(i));
	}

	MatrixXd jacob(3, 4);
	jacob(0, 0) = 0;
	jacob(1, 0) = 0;
	jacob(2, 0) = 1;
	jacob(0, 1) = sa(1)*st(1);
	jacob(1, 1) = -sa(1)*ct(1);
	jacob(2, 1) = ca(1);
	jacob(0, 2) = -ct(1)*(r(2)*st(2) - d(3)*sa(2)*ct(2) + r(3)*ct(3)*st(2) + r(3)*ca(2)*ct(2)*st(3)) - ca(1)*st(1)*(r(2)*ct(2) + d(3)*sa(2)*st(2) + r(3)*ct(2)*ct(3) - r(3)*ca(2)*st(2)*st(3));
	jacob(1, 2) = ca(1)*ct(1)*(r(2)*ct(2) + d(3)*sa(2)*st(2) + r(3)*ct(2)*ct(3) - r(3)*ca(2)*st(2)*st(3)) - st(1)*(r(2)*st(2) - d(3)*sa(2)*ct(2) + r(3)*ct(3)*st(2) + r(3)*ca(2)*ct(2)*st(3));
	jacob(2, 2) = sa(1)*(r(2)*ct(2) + d(3)*sa(2)*st(2) + r(3)*ct(2)*ct(3) - r(3)*ca(2)*st(2)*st(3));
	jacob(0, 3) = ca(1)*st(1)*(r(3)*st(2)*st(3) - r(3)*ca(2)*ct(2)*ct(3)) - ct(1)*(r(3)*ct(2)*st(3) + r(3)*ca(2)*ct(3)*st(2)) + r(3)*sa(1)*sa(2)*ct(3)*st(1);
	jacob(1, 3) = -st(1)*(r(3)*ct(2)*st(3) + r(3)*ca(2)*ct(3)*st(2)) - ca(1)*ct(1)*(r(3)*st(2)*st(3) - r(3)*ca(2)*ct(2)*ct(3)) - r(3)*sa(1)*sa(2)*ct(1)*ct(3);
	jacob(2, 3) = r(3)*ca(1)*sa(2)*ct(3) - sa(1)*(r(3)*st(2)*st(3) - r(3)*ca(2)*ct(2)*ct(3));
	for (int i = 0;i<3; i++)
		for(int j=0; j<4; j++)
			if (jacob(i, j)*jacob(i,j) < 0.00000001) {
				jacob(i, j) = 0;
			}
	return jacob;

}
Matrix4d forwardKinematics2(Matrix4d DH) {
	Matrix4d transform, tmp;
	Vector4d d, theta, r, alpha;
	double ca, ct, sa, st;
	d = DH.col(0);
	theta = DH.col(1);
	r = DH.col(2);
	alpha = DH.col(3);
	transform.setIdentity(); //first transformation = I
	for (int i = 0; i<4; i++) {
		ca = cos(alpha(i));
		ct = cos(theta(i));
		sa = sin(alpha(i));
		st = sin(theta(i));

		tmp <<	ct,	-ca*st,	sa*st,	r(i)*ct,
				st,	ca*ct,	-sa*ct,	r(i)*st,
				0,		sa,			ca,			d(i),
				0,		0,			0,				1;
		transform = transform * tmp;
	}
	return transform;
}

Matrix4d forwardKinematics3(MatrixXd DH, double* actPositions) {
	Matrix4d transform, tmp;
	Vector4d d, theta, r, alpha;
	double ca, ct, sa, st;
	DH(1, 0) = actPositions[2]; //d1
	DH(2, 0) = actPositions[3]; //d2
	DH(2, 1) = actPositions[1]; //t2
	DH(3, 1) = actPositions[0]; //t3
	d = DH.col(0);
	theta = DH.col(1);
	r = DH.col(2);
	alpha = DH.col(3);
	transform.setIdentity(); //first transformation = I
	for (int i = 0; i<4; i++) {
		ca = cos(alpha(i));
		ct = cos(theta(i));
		sa = sin(alpha(i));
		st = sin(theta(i));

		tmp << ct, -ca * st, sa*st, r(i)*ct,
			st, ca*ct, -sa * ct, r(i)*st,
			0, sa, ca, d(i),
			0, 0, 0, 1;
		transform = transform * tmp;
	}

	return transform;
}


Matrix<double, 4, Dynamic> getVelocities(MatrixXd points, Matrix4d DH, double* jointsPos1, double T, double dt) {
	MatrixXd J(3,4), pinv(4, 3); //pseudoinverse
	double distance, T_total;
	Vector4d q,q_prev; //Joint position
	Vector3d dp; //q_e - q_i, difference in start and end pos 
	Vector3d a0, a1, a2, a3, a4, a5; //Velocity profile coefficients
	Matrix4d tForw;
	MatrixXd velJoints(4, 50000), velCartesian(3, 50000), pos(3,50000);
	velJoints.setZero();
	velCartesian.setZero();
	pos.setZero();

	DH(1, 0) = jointsPos1[2]; //d1
	DH(2, 0) = jointsPos1[3]; //d2
	DH(2, 1) = jointsPos1[1]; //t2
	DH(3, 1) = jointsPos1[0]; //t3

	int len = points.rows();
	int prevEnd = 0;
	for (int j = 1; j < len; j++) {
		q << DH(1, 0), DH(2, 0), DH(2, 1), DH(3, 1); //push DH into q
		J = jacobian(DH);
		pinv = pseudoInverse(J);

		//Calculate time to travel between points
		dp = points.row(j) - points.row(j - 1);
		distance = sqrt(dp.dot(dp)); //Distancae squared
		T_total = T * distance; //time to travel between adjacent points

		//Find polynomial coefficients
		a0 = points.row(j-1); //q_i, starting position
		a1 << 0, 0, 0;
		a2 << 0, 0, 0;
		a3 = 10 * dp / pow(T_total, 3);
		a4 = -1.5 * a3 / T_total;
		a5 = -0.4*a4 / T_total;

		double t = 0;
		int numPoints = (int)(double)round(T_total / dt);
		for (int i = 0; i < numPoints + 1; i++) {
			//Calculate new velocities
			q_prev = q;
			velCartesian.col(prevEnd+i) = a1 + 2 * a2*t + 3 * a3*pow(t, 2) + 4 * a4*pow(t, 3) + 5 * a5*pow(t, 4);
			t += dt;
			velJoints.col(prevEnd+i) = pinv * velCartesian.col(prevEnd+i);
			q = velJoints.col(prevEnd+i)*dt + q_prev;
			
			//Store data
			tForw = forwardKinematics2(DH);
			pos.col(prevEnd+ i) = tForw.block<3, 1>(0, 3);
			DH(1, 0) = q(0);
			DH(2, 0) = q(1);
			DH(2, 1) = q(2);
			DH(3, 1) = q(3); //Push new q into DH

			//Update for next loop
			J = jacobian(DH);
			pinv = pseudoInverse(J);
		}
		prevEnd += numPoints + 1;
		//tForw = forwardKinematics2(DH);
	}

	//Remove redundant columns
	int breakPoint = 0;
	int colCount = pos.cols();

	for (int i = 1; i < colCount; i++) { //NOT STARTING AT FIRST LETTER
		if (pos.col(i).any() == 0) {
			breakPoint = i;
			break;
		}
	}
	Matrix<double, 3, Dynamic> posOut;
	posOut = pos.block(0, 0, 3, breakPoint);
	string name = "lineSimPos.csv";
	ofstream myFile(name.c_str());
	myFile << posOut.format(CSVtmp);

	colCount = velJoints.cols();
	for (int i = 1; i < colCount-2; i++) { //NOT STARTING AT FIRST LETTER
		if (velJoints.col(i).any() == 0 && velJoints.col(i+1).any() == 0 && velJoints.col(i+2).any() == 0) {
			breakPoint = i;
			break;
		}
	}	
	cout << velJoints.col(200);
	velJoints.row(0) = velJoints.row(0) / 25.4 * 60;//d1
	velJoints.row(1) = velJoints.row(1) / 12 * 60;
	velJoints.row(2) = velJoints.row(2) / M_PI * 30;
	velJoints.row(3) = velJoints.row(3) / M_PI * 30;
	cout << velJoints.col(200);
	name = "lineVelSim.csv";
	ofstream myFile2(name.c_str());
	myFile2 << velJoints.block(0,0,4,breakPoint).format(CSVtmp);

	return velJoints.block(0, 0, 4, breakPoint); //Return block of actual coords
}

Matrix<int, 4, Dynamic> decimaliseVelocities(Matrix<double, 4, Dynamic> decVelocities, double calcTstep, double dt, double* jointPos1, Matrix4d DH) {
	int numRows = decVelocities.rows();
	int numCols = decVelocities.cols();
	double tmp = calcTstep / dt;
	int numSteps = (int)tmp;
	double decVel;
	int baseVel;
	int fracVel;
	MatrixXi intVels(4,numCols*numSteps);
	MatrixXd pos(3, numCols*numSteps), dVels(4,numCols*numSteps);
	Vector4d q, q_prev, qdotd;
	Vector4i qdoti;
	DH(1, 0) = jointPos1[2]; //d1
	DH(2, 0) = jointPos1[3]; //d2
	DH(2, 1) = jointPos1[1]; //t2
	DH(3, 1) = jointPos1[0]; //t3
	q << DH(1, 0), DH(2, 0), DH(2, 1), DH(3, 1); //push DH into q
	intVels.setZero();
	for (int i = 0; i < numCols; i++) {
		for (int j = 0; j < numRows; j++) {
			decVel = decVelocities(j,i);
			baseVel = (int)floor(decVel);
			decVel -= baseVel;
			decVel *= 10;
			fracVel = (int)round(decVel);
			
			for (int k = 0; k < numSteps/10; k++) {
				for (int l = 0; l < 10; l++) {
					if (l < fracVel){
						intVels(j, 20 * i + 10 * k + l) = baseVel + 1;
					}
					else {
						intVels(j, 20 * i + 10 * k + l) = baseVel;
					}

				}
			}

		}
	}
	for (int i = 0; i < numSteps*numCols; i++) {
		q_prev = q;
		qdoti = intVels.col(i);
		qdotd = qdoti.cast<double>();
		q = qdotd*dt + q_prev;
		pos.col(i) = forwardKinematics2(DH).block<3, 1>(0, 3);
		DH(1, 0) = q(0);
		DH(2, 0) = q(1);
		DH(2, 1) = q(2);
		DH(3, 1) = q(3); //Push new q into DH
	}
	
	dVels = intVels.cast<double>();
	writeToCSVfile2("flinePos.csv", pos);
	writeToCSVfile2("flineVel.csv", dVels);
	getchar();
	return intVels;
}



