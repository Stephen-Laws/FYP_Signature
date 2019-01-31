#pragma once
#include "stdafx.h"
#include <iostream>
#include <Eigen/Dense>
#include<Eigen/Core>
#include<Eigen/SVD>
#include <fstream>
using namespace Eigen;
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

void writeToCSVfile(string name, MatrixXd matrix)
{
	ofstream myFile(name.c_str());
	myFile << matrix.format(CSVFormat);
}


MatrixXd getCoord() {
	MatrixXd localCo(9, 3);
	localCo <<	0, 0, 0, //Topleft Coord
				0.5, 0, 0,
				1, 0, 0,
				0, -1, 0,
				0.5, -1, 0,
				1, -1, 0,
				0, -2, 0,
				0.5, -2, 0,
				1, -2, 0; //Bottomright coord
	return localCo;
}

double dist(Vector3d p1, Vector3d p2) {//Distance between two points
	Vector3d dx = p1 - p2;
	return sqrt(dx.dot(dx));
}

MatrixXd scaleString(MatrixXd points, Vector3d p1, Vector3d p2) {//Scale string based on provisional position
	double len;
	len = dist(p1, p2);
	double endx = points(points.rows() -1, 0);
	double factor = len / endx;
	points.col(0) *= factor;
	points.col(1) *= factor;
	points.col(2) *= factor;
	return points;
}

MatrixXd Coords(char *textPrint) {//Starter coords of letters
	Matrix<double, 9, 3> co; //Local coords
	MatrixXd u(1, 3);
	MatrixXd end(1, 3);
	int maxLen = 200;
	u << 0, 0, 0.3;
	end << 0.5, 0, 1;
	co = getCoord();
	char c;
	int i =0;
	MatrixXd XYCoords(maxLen, 3);
	XYCoords = XYCoords.setZero();
	while (c = *textPrint++ ) {
		switch (c) {
		case 'A':
			XYCoords.row(i++) += co.row(6) + u;
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8) + u;
			XYCoords.row(i++) += co.row(3) + u;
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(5) + u;
			break;
		case 'B':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(6)+u;
			XYCoords.row(i++) += co.row(7)+u;
			XYCoords.row(i++) += co.row(7);
			XYCoords.row(i++) += co.row(1);
			XYCoords.row(i++) += co.row(1)+u;
			XYCoords.row(i++) += co.row(4)+u;
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(5)+u;
			break;
		case 'C':
			XYCoords.row(i++) += co.row(2) + u;
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8) + u;
			XYCoords.row(i++) += co.row(3) + u;
			break;
		case 'D':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(6)+u;
			XYCoords.row(i++) += co.row(7)+u;
			XYCoords.row(i++) += co.row(7);
			XYCoords.row(i++) += co.row(1);
			XYCoords.row(i++) += co.row(1)+u;
			break;
		case 'E':
			XYCoords.row(i++) += co.row(2) + u;
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8) + u;
			XYCoords.row(i++) += co.row(3) + u;
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(4) + u;
			break;
		case 'F':
			XYCoords.row(i++) += co.row(2) + u;
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(6)+u;
			XYCoords.row(i++) += co.row(3) + u;
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(4)+u;
			break;
		case 'G':
			XYCoords.row(i++) += co.row(2) + u;
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(4)+u;
			break;
		case 'H':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(6)+u;
			XYCoords.row(i++) += co.row(3)+u;
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(5) + u;
			XYCoords.row(i++) += co.row(2)+u;
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8) + u;
			break;
		case 'I':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(6)+u;
			break;
		case 'J':
			XYCoords.row(i++) += co.row(2) + u;
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(3)+u;
			break;
		case 'K':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(6)+u;
			XYCoords.row(i++) += co.row(3)+u;
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2)+u;
			XYCoords.row(i++) += co.row(4)+u;
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8)+u;
			break;
		case 'L':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8)+u;
			break;
		case 'M':
			XYCoords.row(i++) += co.row(6) + u;
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8) + u;

			break;
		case 'N':
			XYCoords.row(i++) += co.row(6) + u;
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2) + u;
			break;
		case 'O':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(0) + u;
			break;
		case 'P':
			XYCoords.row(i++) += co.row(6) + u;
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(3) + u;
			break;
		case 'Q':
			XYCoords.row(i++) += co.row(8) + u;
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(4)+u;
			break;
		case 'R':
			XYCoords.row(i++) += co.row(6) + u;
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(3) + u;
			XYCoords.row(i++) += co.row(4)+u;
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8) + u;
			break;
		case 'S':
			XYCoords.row(i++) += co.row(2) + u;
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(6)+u;
			break;
		case 'T':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2)+u;
			XYCoords.row(i++) += co.row(1)+u;
			XYCoords.row(i++) += co.row(1);
			XYCoords.row(i++) += co.row(7);
			XYCoords.row(i++) += co.row(7) + u;
			break;
		case 'U':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2)+u;
			break;
		case 'V':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(7);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2)+u;
			break;
		case 'W':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2) + u;
			break;
		case 'X':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8)+u;
			XYCoords.row(i++) += co.row(6)+u;
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2) + u;
			break;
		case 'Y':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(3);
			XYCoords.row(i++) += co.row(5);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(2)+u;
			XYCoords.row(i++) += co.row(4) + u;
			XYCoords.row(i++) += co.row(4);
			XYCoords.row(i++) += co.row(7);
			XYCoords.row(i++) += co.row(7)+u;
			break;
		case 'Z':
			XYCoords.row(i++) += co.row(0);
			XYCoords.row(i++) += co.row(2);
			XYCoords.row(i++) += co.row(6);
			XYCoords.row(i++) += co.row(8);
			XYCoords.row(i++) += co.row(8)+u;
			XYCoords.row(i++) += co.row(8) + u;
			break;
		case ' ':
			break;
		default:
				break;
		}
		XYCoords.row(i++) += co.row(2) + end;
		co.col(0) = co.col(0).array() + 1.5;
	}

	//Remove zero rows
	int breakPoint =0;
	int rowCount = XYCoords.rows();
	for (int i = 4; i < rowCount; i++) { //NOT STARTING AT FIRST LETTER
		if (XYCoords.row(i).any() == 0) {
			breakPoint = i;
			break;
		}
	}
	//cout << XYCoords.block(0, 0, breakPoint, 3) << endl;
	return XYCoords.block(0, 0,breakPoint,3); //Return block of actual coords

}

